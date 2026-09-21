#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
// --
#include <cstring>
#include <filesystem>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/core/status_log.hpp>
#include <movutl/core/vector.hpp>
#include <movutl/gui/gui.hpp>
#include <stdio.h>
#include <vector>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
#include <imgui_internal.h>
#include <memory>
#include <movutl/vulkan/vk_context.hpp>
#include <movutl/vulkan/vk_image.hpp>

// ImGui - GLFW(NO_API) + Vulkan swapchain。デバイスはVkContext(レンダラー/computeと共有)を使う
static void error_callback(int error, const char* description) { fprintf(stderr, "Error %d: %s\n", error, description); }

namespace mu {

namespace {

constexpr int kFramesInFlight = 2;

struct Swapchain {
  VkSurfaceKHR surface = VK_NULL_HANDLE;
  VkSwapchainKHR sc    = VK_NULL_HANDLE;
  VkRenderPass rp      = VK_NULL_HANDLE;
  VkFormat format      = VK_FORMAT_UNDEFINED;
  VkExtent2D extent    = {0, 0};
  uint32_t min_images  = 2;
  std::vector<VkImage> images;
  std::vector<VkImageView> views;
  std::vector<VkFramebuffer> fbs;
  std::vector<VkSemaphore> render_done; // swapchain画像ごと
  VkCommandPool pool = VK_NULL_HANDLE;
  VkCommandBuffer cbs[kFramesInFlight]{};
  VkFence fences[kFramesInFlight]{};
  VkSemaphore acquire[kFramesInFlight]{};
  int frame = 0;
  // 直近に描画した画面のコピー(capture_screen用)。swapchain画像はpresent後の内容が不定なため毎フレームここへコピーする
  // ponytail: 毎フレームGPU内コピーする(スクショ機能のため)。負荷が問題になれば要求があるフレームだけコピーする方式にする
  std::unique_ptr<GpuImage> capture;
} g_sc;

void destroy_swapchain_views(VkDevice d) {
  for(auto fb : g_sc.fbs) vkDestroyFramebuffer(d, fb, nullptr);
  for(auto v : g_sc.views) vkDestroyImageView(d, v, nullptr);
  for(auto s : g_sc.render_done) vkDestroySemaphore(d, s, nullptr);
  g_sc.fbs.clear();
  g_sc.views.clear();
  g_sc.render_done.clear();
}

// ウィンドウサイズに合わせてswapchainを(再)作成する。最小化中(サイズ0)はfalse
bool rebuild_swapchain(GLFWwindow* window) {
  auto* ctx = VkContext::Get();
  auto d    = ctx->device();
  int w = 0, h = 0;
  glfwGetFramebufferSize(window, &w, &h);
  if(w <= 0 || h <= 0) return false;
  {
    std::lock_guard<std::mutex> lock(ctx->queue_mutex());
    vkDeviceWaitIdle(d);
  }

  VkSurfaceCapabilitiesKHR caps;
  vkGetPhysicalDeviceSurfaceCapabilitiesKHR(ctx->physical_device(), g_sc.surface, &caps);
  VkExtent2D ext = caps.currentExtent;
  if(ext.width == 0xFFFFFFFF) ext = {(uint32_t)w, (uint32_t)h};
  if(ext.width == 0 || ext.height == 0) return false;

  uint32_t nf = 0;
  vkGetPhysicalDeviceSurfaceFormatsKHR(ctx->physical_device(), g_sc.surface, &nf, nullptr);
  std::vector<VkSurfaceFormatKHR> formats(nf);
  vkGetPhysicalDeviceSurfaceFormatsKHR(ctx->physical_device(), g_sc.surface, &nf, formats.data());
  VkSurfaceFormatKHR fmt = formats[0];
  for(auto& f : formats)
    if(f.format == VK_FORMAT_B8G8R8A8_UNORM || f.format == VK_FORMAT_R8G8B8A8_UNORM) {
      fmt = f;
      if(f.format == VK_FORMAT_B8G8R8A8_UNORM) break;
    }

  uint32_t count = caps.minImageCount + 1;
  if(caps.maxImageCount > 0 && count > caps.maxImageCount) count = caps.maxImageCount;
  VkSwapchainCreateInfoKHR sci{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
  sci.surface          = g_sc.surface;
  sci.minImageCount    = count;
  sci.imageFormat      = fmt.format;
  sci.imageColorSpace  = fmt.colorSpace;
  sci.imageExtent      = ext;
  sci.imageArrayLayers = 1;
  sci.imageUsage       = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
  sci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
  sci.preTransform     = caps.currentTransform;
  sci.compositeAlpha   = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
  sci.presentMode      = VK_PRESENT_MODE_FIFO_KHR;
  sci.clipped          = VK_TRUE;
  sci.oldSwapchain     = g_sc.sc;
  VkSwapchainKHR nsc   = VK_NULL_HANDLE;
  if(vkCreateSwapchainKHR(d, &sci, nullptr, &nsc) != VK_SUCCESS) {
    LOG_F(ERROR, "vkCreateSwapchainKHR failed");
    return false;
  }
  destroy_swapchain_views(d);
  if(g_sc.sc) vkDestroySwapchainKHR(d, g_sc.sc, nullptr);
  g_sc.sc     = nsc;
  g_sc.extent = ext;

  if(g_sc.rp == VK_NULL_HANDLE) {
    g_sc.format = fmt.format;
    VkAttachmentDescription att{};
    att.format         = fmt.format;
    att.samples        = VK_SAMPLE_COUNT_1_BIT;
    att.loadOp         = VK_ATTACHMENT_LOAD_OP_CLEAR;
    att.storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
    att.stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    att.initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED;
    att.finalLayout    = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL; // 描画後にcapture用コピー→presentへ遷移する
    VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription sub{};
    sub.pipelineBindPoint    = VK_PIPELINE_BIND_POINT_GRAPHICS;
    sub.colorAttachmentCount = 1;
    sub.pColorAttachments    = &ref;
    VkSubpassDependency dep{VK_SUBPASS_EXTERNAL, 0, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, 0};
    VkRenderPassCreateInfo rpi{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    rpi.attachmentCount = 1;
    rpi.pAttachments    = &att;
    rpi.subpassCount    = 1;
    rpi.pSubpasses      = &sub;
    rpi.dependencyCount = 1;
    rpi.pDependencies   = &dep;
    vkCreateRenderPass(d, &rpi, nullptr, &g_sc.rp);
  }

  uint32_t n = 0;
  vkGetSwapchainImagesKHR(d, nsc, &n, nullptr);
  g_sc.images.resize(n);
  vkGetSwapchainImagesKHR(d, nsc, &n, g_sc.images.data());
  g_sc.min_images = caps.minImageCount;
  for(auto img : g_sc.images) {
    VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    vi.image                       = img;
    vi.viewType                    = VK_IMAGE_VIEW_TYPE_2D;
    vi.format                      = g_sc.format;
    vi.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    vi.subresourceRange.levelCount = 1;
    vi.subresourceRange.layerCount = 1;
    VkImageView view;
    vkCreateImageView(d, &vi, nullptr, &view);
    g_sc.views.push_back(view);
    VkFramebufferCreateInfo fi{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
    fi.renderPass      = g_sc.rp;
    fi.attachmentCount = 1;
    fi.pAttachments    = &view;
    fi.width           = ext.width;
    fi.height          = ext.height;
    fi.layers          = 1;
    VkFramebuffer fb;
    vkCreateFramebuffer(d, &fi, nullptr, &fb);
    g_sc.fbs.push_back(fb);
    VkSemaphoreCreateInfo semi{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    VkSemaphore sem;
    vkCreateSemaphore(d, &semi, nullptr, &sem);
    g_sc.render_done.push_back(sem);
  }
  g_sc.capture = std::make_unique<GpuImage>(ext.width, ext.height, g_sc.format, VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);
  g_sc.capture->set_rest_layout(VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
  return true;
}

} // namespace

void GUIManager::init() {
  glfwSetErrorCallback(error_callback);

#if defined(GLFW_VERSION_MAJOR) && (GLFW_VERSION_MAJOR > 3 || GLFW_VERSION_MINOR >= 4)
  glfwInitVulkanLoader(vkGetInstanceProcAddr); // GLFWが別のloader(MoltenVK直接等)を掴んでinstanceが食い違うのを防ぐ
#endif
  if(!glfwInit()) {
    LOG_F(ERROR, "Could not initialize GLFW");
    return;
  }

  if(!glfwVulkanSupported()) {
    LOG_F(FATAL, "Vulkanが利用できません(loader/ICDを確認してください)");
    return;
  }
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

  LOG_F(2, "ImGui::CheckVersion();");
  IMGUI_CHECKVERSION();
  LOG_F(2, "ImGui::CreateContext with docking enabled");
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  // start docking
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable; // | ImGuiConfigFlags_ViewportsEnable;
  io.ConfigFlags |= ImGuiWindowFlags_NoBackground;
  // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
  io.ConfigDockingWithShift = false;
  io.ConfigDockingNoSplit   = false;

  glfw_window = glfwCreateWindow(1280, 720, "movutl", NULL, NULL);

  uint32_t n_ext         = 0;
  const char** glfw_exts = glfwGetRequiredInstanceExtensions(&n_ext);
  std::vector<std::string> exts(glfw_exts, glfw_exts + n_ext);
  auto* ctx = VkContext::Get();
  if(!ctx->create(exts, true, [](VkInstance i, VkPhysicalDevice pd, uint32_t qf) { return glfwGetPhysicalDevicePresentationSupport(i, pd, qf) == GLFW_TRUE; })) {
    LOG_F(FATAL, "Vulkanデバイスの初期化に失敗しました");
    return;
  }
  if(glfwCreateWindowSurface(ctx->instance(), glfw_window, nullptr, &g_sc.surface) != VK_SUCCESS) {
    LOG_F(FATAL, "Vulkan surfaceの作成に失敗しました");
    return;
  }
  rebuild_swapchain(glfw_window);
  {
    auto d = ctx->device();
    VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pci.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pci.queueFamilyIndex = ctx->queue_family();
    vkCreateCommandPool(d, &pci, nullptr, &g_sc.pool);
    VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cai.commandPool        = g_sc.pool;
    cai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = kFramesInFlight;
    vkAllocateCommandBuffers(d, &cai, g_sc.cbs);
    for(int i = 0; i < kFramesInFlight; i++) {
      VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
      fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;
      vkCreateFence(d, &fi, nullptr, &g_sc.fences[i]);
      VkSemaphoreCreateInfo si{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
      vkCreateSemaphore(d, &si, nullptr, &g_sc.acquire[i]);
    }
  }

  // Setup ImGui binding
  ImGui_ImplGlfw_InitForVulkan(glfw_window, true);
  ImGui_ImplVulkan_InitInfo vi{};
  vi.Instance           = ctx->instance();
  vi.PhysicalDevice     = ctx->physical_device();
  vi.Device             = ctx->device();
  vi.QueueFamily        = ctx->queue_family();
  vi.Queue              = ctx->queue();
  vi.DescriptorPoolSize = 256; // フォント + Viewer等のAddTexture分
  vi.RenderPass         = g_sc.rp;
  vi.MinImageCount      = g_sc.min_images;
  vi.ImageCount         = (uint32_t)g_sc.images.size();
  vi.MSAASamples        = VK_SAMPLE_COUNT_1_BIT;
  ImGui_ImplVulkan_Init(&vi);
  glfwSetDropCallback(glfw_window, [](GLFWwindow*, int n, const char** paths) {
    for(int i = 0; i < n; i++) {
      LOG_F(INFO, "drop: %s", paths[i]);
      open_file(paths[i]);
    }
  });
  LOG_F(INFO, "glfw drop callback registered");

  const float fontSize      = 18.0f;
  auto font_path            = fs_get_font_path();
  std::string font_fnames[] = {
    font_path + "/Meiryo.ttf",
    font_path + "/fa-solid-900.ttf",
  };

  auto font      = io.Fonts->AddFontDefault();
  font->Scale    = 1.0f;
  io.FontDefault = io.Fonts->AddFontFromFileTTF(font_fnames[0].c_str(), fontSize, nullptr, io.Fonts->GetGlyphRangesJapanese());

  ImFontConfig config;
  config.MergeMode                    = true;
  config.MergeMode                    = true;
  config.GlyphMinAdvanceX             = fontSize;
  static const ImWchar icons_ranges[] = {ICON_MIN_FA, ICON_MAX_FA, 0};
  io.Fonts->AddFontFromFileTTF(font_fnames[1].c_str(), fontSize * 0.8, &config, icons_ranges);
  LOG_F(2, "[imgui] Building font %s %f", font_fnames[1].c_str(), fontSize);
  io.Fonts->Build();

  detail::init_gui_panels();
}

ImVec4 clear_color = ImColor(20, 20, 20);

namespace detail {

void gui_new_frame() {
  MOVUTL_ZONE_SCOPED_N("gui_new_frame");
  auto window = GUIManager::Get()->glfw_window;

  bool should_close               = glfwWindowShouldClose(window);
  GUIManager::Get()->should_close = should_close;

  glfwPollEvents();
  {
    // タイトルは変化したときだけ更新する(毎フレームのglfwSetWindowTitleを避ける)
    static std::string last_title;
    auto pj           = Project::Get();
    std::string title = "movutl - " + ((pj && !pj->path.empty()) ? std::filesystem::path(pj->path).filename().string() : std::string("(無題)"));
    if(status_log_dirty()) title += " *";
    if(title != last_title) {
      glfwSetWindowTitle(window, title.c_str());
      last_title = title;
    }
  }
  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  render_main_menu_bar();

  // メインメニューバーとステータスバー(固定フッター)を除く領域をドッキングスペースとする
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  const float footer_height     = ImGui::GetFrameHeight();
  const ImVec2 dock_pos         = viewport->WorkPos;
  const ImVec2 dock_size(viewport->WorkSize.x, viewport->WorkSize.y - footer_height);

  static ImGuiID main_dockspace_id = 0;
  if(main_dockspace_id == 0) main_dockspace_id = ImGui::GetID("MainDockSpace");

  ImGui::SetNextWindowPos(dock_pos);
  ImGui::SetNextWindowSize(dock_size);
  ImGui::SetNextWindowViewport(viewport->ID);

  const auto c = ImGui::GetStyle().Colors;
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(c[ImGuiCol_WindowBg].x, c[ImGuiCol_WindowBg].y, c[ImGuiCol_WindowBg].z, 0.0f));
  ImGui::PushStyleColor(ImGuiCol_DockingEmptyBg, ImVec4(c[ImGuiCol_DockingEmptyBg].x, c[ImGuiCol_DockingEmptyBg].y, c[ImGuiCol_DockingEmptyBg].z, 0.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
  ImGui::Begin("##MainDockHost", nullptr, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground);
  ImGui::PopStyleVar(3);
  auto app          = GUIManager::Get();
  app->dockspace_id = ImGui::DockSpace(main_dockspace_id, ImVec2(0.0f, 0.0f));
  ImGui::End();
  ImGui::PopStyleColor(2);

  render_status_bar();

  // レイアウトが空(DockSpaceOverViewportが自動生成した空の中央ノードしかない)なら未初期化とみなす
  const auto dock_node       = ImGui::DockBuilderGetNode(app->dockspace_id);
  const bool is_empty_layout = dock_node == nullptr || (dock_node->IsCentralNode() && dock_node->Windows.empty() && dock_node->ChildNodes[0] == nullptr);

  // ワークスペースの遅延適用 / 初回起動時(iniにレイアウト未保存)はデフォルトレイアウトを適用
  if(!app->pending_workspace.empty()) {
    const std::string name = app->pending_workspace;
    app->pending_workspace.clear();
    apply_workspace(name.c_str());
  } else if(is_empty_layout) {
    apply_workspace("Default");
  }
}

void gui_render_to_screen() {
  MOVUTL_ZONE_SCOPED_N("gui_render_to_screen");
  auto window = GUIManager::Get()->glfw_window;
  auto* ctx   = VkContext::Get();
  auto d      = ctx->device();
  ImGui::Render();
  ImDrawData* draw_data = ImGui::GetDrawData();

  int fw = 0, fh = 0;
  glfwGetFramebufferSize(window, &fw, &fh);
  if(fw <= 0 || fh <= 0) return; // 最小化中は描画しない
  if(g_sc.sc == VK_NULL_HANDLE || (uint32_t)fw != g_sc.extent.width || (uint32_t)fh != g_sc.extent.height) {
    if(!rebuild_swapchain(window)) return;
    ImGui_ImplVulkan_SetMinImageCount(g_sc.min_images);
  }

  const int f = g_sc.frame;
  vkWaitForFences(d, 1, &g_sc.fences[f], VK_TRUE, UINT64_MAX);
  uint32_t idx = 0;
  VkResult r   = vkAcquireNextImageKHR(d, g_sc.sc, UINT64_MAX, g_sc.acquire[f], VK_NULL_HANDLE, &idx);
  if(r == VK_ERROR_OUT_OF_DATE_KHR) {
    rebuild_swapchain(window);
    return;
  }
  vkResetFences(d, 1, &g_sc.fences[f]);
  auto cb = g_sc.cbs[f];
  vkResetCommandBuffer(cb, 0);
  VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
  bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  vkBeginCommandBuffer(cb, &bi);
  VkClearValue clear{};
  clear.color = {{clear_color.x, clear_color.y, clear_color.z, clear_color.w}};
  VkRenderPassBeginInfo rbi{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
  rbi.renderPass      = g_sc.rp;
  rbi.framebuffer     = g_sc.fbs[idx];
  rbi.renderArea      = {{0, 0}, g_sc.extent};
  rbi.clearValueCount = 1;
  rbi.pClearValues    = &clear;
  vkCmdBeginRenderPass(cb, &rbi, VK_SUBPASS_CONTENTS_INLINE);
  ImGui_ImplVulkan_RenderDrawData(draw_data, cb);
  vkCmdEndRenderPass(cb);

  // 描画結果をcapture画像へコピーしてからpresentへ遷移(render passのfinalLayoutはTRANSFER_SRC)
  g_sc.capture->transition(cb, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
  VkImageCopy cp{};
  cp.srcSubresource = cp.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
  cp.extent                             = {g_sc.extent.width, g_sc.extent.height, 1};
  vkCmdCopyImage(cb, g_sc.images[idx], VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, g_sc.capture->image(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &cp);
  g_sc.capture->transition(cb, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
  VkImageMemoryBarrier pb{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
  pb.oldLayout           = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
  pb.newLayout           = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
  pb.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  pb.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  pb.image               = g_sc.images[idx];
  pb.subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
  pb.srcAccessMask       = VK_ACCESS_TRANSFER_READ_BIT;
  vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, 0, nullptr, 0, nullptr, 1, &pb);
  vkEndCommandBuffer(cb);

  const VkPipelineStageFlags wait_stage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
  si.waitSemaphoreCount   = 1;
  si.pWaitSemaphores      = &g_sc.acquire[f];
  si.pWaitDstStageMask    = &wait_stage;
  si.commandBufferCount   = 1;
  si.pCommandBuffers      = &cb;
  si.signalSemaphoreCount = 1;
  si.pSignalSemaphores    = &g_sc.render_done[idx];
  VkPresentInfoKHR pi{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
  pi.waitSemaphoreCount = 1;
  pi.pWaitSemaphores    = &g_sc.render_done[idx];
  pi.swapchainCount     = 1;
  pi.pSwapchains        = &g_sc.sc;
  pi.pImageIndices      = &idx;
  VkResult pr;
  {
    std::lock_guard<std::mutex> lock(ctx->queue_mutex());
    vkQueueSubmit(ctx->queue(), 1, &si, g_sc.fences[f]);
    pr = vkQueuePresentKHR(ctx->queue(), &pi);
  }
  if(pr == VK_ERROR_OUT_OF_DATE_KHR || pr == VK_SUBOPTIMAL_KHR) rebuild_swapchain(window);
  g_sc.frame = (f + 1) % kFramesInFlight;
}

} // namespace detail

Ref<Image> capture_screen() {
  if(!g_sc.capture) return nullptr;
  std::vector<uint8_t> raw;
  if(!g_sc.capture->readback_raw(raw)) return nullptr; // 直近に描画された(=表示中の)画面。ImGui描画済みのUI込み
  const int w = (int)g_sc.capture->width(), h = (int)g_sc.capture->height();
  auto img        = cutil::make_ref<Image>(w, h);
  img->has_alpha  = false;
  auto* dst       = reinterpret_cast<uint8_t*>(img->data());
  const bool bgra = g_sc.format == VK_FORMAT_B8G8R8A8_UNORM;
  for(size_t i = 0; i < (size_t)w * h; i++) {
    dst[i * 4 + 0] = raw[i * 4 + (bgra ? 2 : 0)];
    dst[i * 4 + 1] = raw[i * 4 + 1];
    dst[i * 4 + 2] = raw[i * 4 + (bgra ? 0 : 2)];
    dst[i * 4 + 3] = 255;
  }
  img->dirty();
  return img;
}

void update() {
  MOVUTL_ZONE_SCOPED_N("mu::update");
  {
    MOVUTL_ZONE_SCOPED_N("sleep");
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
  }
  { // gui thread
    detail::gui_new_frame();
    detail::process_command_shortcuts();
    detail::AppMain::Get()->update_frame_impl();
    tick_running_commands();
    detail::update_gui_panels();
    detail::gui_render_to_screen();
  }

  { // レンダーワーカーへのenqueueとキャッシュ済みフレームの取得のみ(実際のレンダリングはバックグラウンドスレッドで行う)
    detail::update_renderer_thread();
    detail::update_audio_thread();
  }
  MOVUTL_FRAME_MARK;
}

void GUIManager::terminate() {
  detail::AppMain::Get()->render_pool.stop();
  detail::AppMain::Get()->audio_worker.stop();
  detail::AppMain::Get()->audio_player.stop();
  auto* ctx = VkContext::Get();
  if(ctx->valid()) {
    ctx->wait_idle();
    vkDeviceWaitIdle(ctx->device());
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    auto d = ctx->device();
    g_sc.capture.reset();
    destroy_swapchain_views(d);
    for(int i = 0; i < kFramesInFlight; i++) {
      vkDestroyFence(d, g_sc.fences[i], nullptr);
      vkDestroySemaphore(d, g_sc.acquire[i], nullptr);
    }
    vkDestroyCommandPool(d, g_sc.pool, nullptr);
    vkDestroyRenderPass(d, g_sc.rp, nullptr);
    vkDestroySwapchainKHR(d, g_sc.sc, nullptr);
    vkDestroySurfaceKHR(ctx->instance(), g_sc.surface, nullptr);
  }
  glfwTerminate();
}

void terminate() { GUIManager::Get()->terminate(); }

bool should_terminate() { return GUIManager::Get()->should_close; }

GUIManager* GUIManager::singleton_ = nullptr;

} // namespace mu
