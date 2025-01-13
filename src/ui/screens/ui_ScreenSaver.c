// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 9.1.0
// Project name: ESPGrind

#include "../ui.h"

void ui_ScreenSaver_screen_init(void)
{
ui_ScreenSaver = lv_obj_create(NULL);
lv_obj_remove_flag( ui_ScreenSaver, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM );    /// Flags
lv_obj_set_flex_flow(ui_ScreenSaver,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_ScreenSaver, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

ui_standbyLabel = lv_label_create(ui_ScreenSaver);
lv_obj_set_width( ui_standbyLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_standbyLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_standbyLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_standbyLabel,"Standby");
lv_obj_add_flag( ui_standbyLabel, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_remove_flag( ui_standbyLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_standbyLabel, &lv_font_montserrat_48, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_standbyInfoLabel = lv_label_create(ui_ScreenSaver);
lv_obj_set_width( ui_standbyInfoLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_standbyInfoLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_standbyInfoLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_standbyInfoLabel,"Touch to resume");
lv_obj_add_flag( ui_standbyInfoLabel, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_remove_flag( ui_standbyInfoLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags


}
