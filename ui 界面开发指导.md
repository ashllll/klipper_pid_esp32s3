1. 概述
本规则旨在指导 AI 使用 Astra UI Lite 库在 ESP32-S3 平台上（配合 128x64 OLED 显示屏）开发用户界面。Astra UI Lite 是一个轻量级的、基于层级列表的 UI 框架。

2. 核心概念
层级列表 (Hierarchical List): UI 由 astra_list_item_t 结构体构成的树状层级列表组成。每个节点代表一个菜单项或 UI 元素。
项类型 (Item Types): 支持多种项类型：普通列表项 (list_item)、开关 (switch_item)、滑块 (slider_item) 和用户自定义项 (user_item)。
选择器 (Selector): astra_selector_t 用于导航列表，高亮显示当前选中的项，并处理进入/退出子菜单或触发动作。
绘图驱动 (Draw Driver): astra_ui_draw_driver.h 定义了绘图函数的宏接口（HAL）。必须根据 ESP32-S3 使用的图形库（如 U8g2）实现这些宏。
绘制器 (Drawer): astra_ui_drawer.h 中的函数负责调用绘图驱动宏来绘制具体的 UI 元素。
核心 (Core): astra_ui_core.h 提供初始化、刷新和主循环处理函数。
3. 开发规则
3.1. 环境设置与初始化
包含头文件: 包含必要的 Astra UI 头文件 (astra_ui_core.h, astra_ui_item.h 等) 以及 ESP32-S3 平台和所选图形库（如 U8g2）的头文件。
实现绘图驱动宏:
在 astra_ui_draw_driver.c 或单独的配置文件中，必须使用 #define 重新定义 astra_ui_draw_driver.h 中所有以 oled_ 开头的宏。
将这些宏映射到 ESP32-S3 平台上所使用的图形库（如 U8g2）的对应函数。例如：
// 假设使用 U8g2 库
#include "u8g2.h"
extern u8g2_t u8g2; // 假设 u8g2 实例已在别处定义和初始化

#define oled_set_font(font) u8g2_SetFont(&u8g2, font)
#define oled_draw_str(x, y, str) u8g2_DrawStr(&u8g2, x, y, str)
#define oled_draw_UTF8(x, y, str) u8g2_DrawUTF8(&u8g2, x, y, str)
#define oled_get_str_width(str) u8g2_GetStrWidth(&u8g2, str)
// ... 实现所有其他 oled_ 宏 ...
#define oled_send_buffer() u8g2_SendBuffer(&u8g2)
#define oled_clear_buffer() u8g2_ClearBuffer(&u8g2)
// 实现延时和获取 tick 函数
#define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms)) // 示例：使用 FreeRTOS 延时
#define get_ticks() xTaskGetTickCount()       // 示例：使用 FreeRTOS 获取 tick
关键: 确保所有绘图相关的宏都被正确实现，否则 UI 无法显示。
初始化图形库: 在调用 Astra UI 初始化之前，完成 ESP32-S3 硬件（SPI/I2C）和图形库（如 U8g2）的初始化。
初始化 Astra UI 驱动: 调用 astra_ui_driver_init() (如果该函数在 astra_ui_draw_driver.c 中有具体实现，用于初始化驱动相关的状态)。
设置字体: 调用 astra_set_font() 并传入图形库支持的字体指针。例如 astra_set_font(u8g2_font_ncenB08_tr);。
初始化 Astra UI 核心: 调用 astra_init_core()。
初始化列表: 调用 astra_init_list() 来初始化根列表。
3.2. 构建 UI 结构
获取根列表: 使用 astra_get_root_list() 获取根列表项指针。
创建列表项:
使用 astra_new_list_item("文本") 创建普通列表项。
使用 astra_new_switch_item("文本", &bool_variable) 创建开关项，传入布尔变量的指针。
使用 astra_new_slider_item("文本", &int_variable, step, min, max) 创建滑块项，传入整型变量指针及步进、范围。
使用 astra_new_user_item("文本", init_func, loop_func, exit_func) 创建用户自定义项，传入相应的函数指针。
组织层级: 使用 astra_push_item_to_list(parent_item_ptr, child_item_ptr) 将创建的项添加到父项中，构建菜单层级。
示例:

// 全局变量
bool wifi_enabled = false;
int16_t brightness = 50;

// 自定义项函数
void custom_screen_init() { /* ... */ }
void custom_screen_loop() { /* ... */ }
void custom_screen_exit() { /* ... */ }

void build_ui() {
    astra_list_item_t *root = astra_get_root_list();
    astra_list_item_t *settings_menu = astra_new_list_item("Settings");
    astra_list_item_t *display_menu = astra_new_list_item("Display");

    // 将 Settings 菜单添加到根列表
    astra_push_item_to_list(root, settings_menu);

    // 添加开关项到 Settings 菜单
    astra_push_item_to_list(settings_menu, astra_new_switch_item("Enable WiFi", &wifi_enabled));

    // 添加 Display 子菜单到 Settings 菜单
    astra_push_item_to_list(settings_menu, display_menu);

    // 添加滑块项到 Display 菜单
    astra_push_item_to_list(display_menu, astra_new_slider_item("Brightness", &brightness, 5, 0, 100));

    // 添加自定义项到根列表
    astra_push_item_to_list(root, astra_new_user_item("Custom Screen", custom_screen_init, custom_screen_loop, custom_screen_exit));
}
3.3. 主循环与交互
UI 主处理: 在 ESP32-S3 的主循环或专用任务中，定期调用 ad_astra()。此函数处理输入（需要自行实现输入读取并传递给 Astra UI，这部分未在提供的代码中明确，可能需要查看 astra_ui_core.c 或示例）、更新 UI 状态、计算动画并触发绘制。
输入处理: (需要根据 astra_ui_core.c 的具体实现来确定) 通常需要读取按键（上、下、确认、返回）状态，并将这些事件传递给 Astra UI 的某个函数（可能未在头文件中声明，或者通过 ad_astra() 内部处理）。
绘制: ad_astra() 函数内部会调用 astra_ui_main_core() 或 astra_ui_widget_core()，这些函数进而调用 astra_draw_... 函数，最终通过绘图驱动宏将 UI 绘制到屏幕缓冲区，并通过 oled_send_buffer() 发送到 OLED 显示屏。
3.4. 使用特定项类型
Switch Item: UI 会自动处理开关状态的切换。只需确保传入的布尔变量指针有效。
Slider Item: UI 会处理滑块值的调整（通常通过左右键或旋转编码器，输入处理需自行实现）。确认值（通常通过确认键）后，is_confirmed 标志会被设置，value_backup 会更新。
User Item:
当用户选择并进入 user_item 时，init_function 会被调用一次。
之后在每次 ad_astra() 调用且该 user_item 处于活动状态时，loop_function 会被调用。开发者在此函数中实现自定义界面的绘制和逻辑。
当用户退出 user_item 时，exit_function 会被调用一次。
注意: 在 loop_function 中绘制自定义界面时，仍需使用已实现的 oled_ 绘图宏。
3.5. 信息栏与弹窗
使用 astra_push_info_bar("消息", duration_ms) 显示临时信息。
使用 astra_push_pop_up("消息", duration_ms) 显示弹出提示。
这些通常用于显示操作结果或状态提示。
4. ESP32-S3 特定注意事项
任务管理: 建议将 UI 处理（调用 ad_astra()）放在一个单独的 FreeRTOS 任务中，以避免阻塞其他关键任务。
输入处理: 实现 GPIO 中断或轮询来读取按键输入，并将输入事件传递给 Astra UI。
图形库集成: 确保选择的图形库（如 U8g2）与 ESP32-S3 兼容，并正确配置其与 OLED 屏幕的通信接口（SPI 或 I2C）。
资源: 注意 Flash 和 RAM 的使用。Astra UI Lite 本身设计为轻量级，但 UI 的复杂度和字体大小会影响资源消耗。
5. 总结
开发流程：

设置: 初始化硬件、图形库、Astra UI 驱动和核心。
实现驱动: 关键步骤 - 实现 oled_ 宏。
构建: 使用 astra_new_... 和 astra_push_item_to_list 定义 UI 结构。
集成: 将 ad_astra() 调用和输入处理集成到 ESP32-S3 的主循环或任务中。
测试: 编译、烧录并测试 UI 的显示和交互。

## 6. 开发流程图 (Mermaid)

```mermaid
graph TD;
    subgraph 初始化阶段
        direction LR
        A[1. 初始化硬件与图形库 (ESP32-S3 & 图形库如 U8g2)] --> B(2. 实现绘图驱动宏 (定义所有 oled_ 宏));
        B --> C[3. 初始化 Astra UI (驱动, 字体, 核心, 列表)];
    end

    subgraph 开发阶段
        direction LR
        D[4. 构建 UI 结构 (使用 astra_new_... 创建项, 使用 astra_push_item_to_list 组织层级)] --> E[5. 实现输入处理 (读取按键/编码器等输入)];
        E --> F[6. 集成主循环 (在 FreeRTOS 任务中调用 ad_astra())];
    end

    subgraph 测试阶段
        direction LR
        G[7. 测试与调试 (编译, 烧录, 验证 UI 显示和交互)];
    end

    初始化阶段 --> 开发阶段;
    开发阶段 --> 测试阶段;
