// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 9.1.0
// Project name: ESPGrind

#include "../ui.h"

void ui_SettingsMain_screen_init(void)
{
ui_SettingsMain = lv_obj_create(NULL);
lv_obj_remove_flag( ui_SettingsMain, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_scrollbar_mode(ui_SettingsMain, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_scroll_dir(ui_SettingsMain, LV_DIR_VER);
lv_obj_set_flex_flow(ui_SettingsMain,LV_FLEX_FLOW_ROW_WRAP);
lv_obj_set_flex_align(ui_SettingsMain, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

ui_HeaderSettings = lv_obj_create(ui_SettingsMain);
lv_obj_remove_style_all(ui_HeaderSettings);
lv_obj_set_width( ui_HeaderSettings, lv_pct(100));
lv_obj_set_height( ui_HeaderSettings, lv_pct(18));
lv_obj_set_x( ui_HeaderSettings, 0 );
lv_obj_set_y( ui_HeaderSettings, 48 );
lv_obj_set_align( ui_HeaderSettings, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_HeaderSettings,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_HeaderSettings, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
lv_obj_remove_flag( ui_HeaderSettings, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_HeaderSettings, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_pad_left(ui_HeaderSettings, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_HeaderSettings, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_HeaderSettings, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_HeaderSettings, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SettingsExit = lv_button_create(ui_HeaderSettings);
lv_obj_set_width( ui_SettingsExit, 36);
lv_obj_set_height( ui_SettingsExit, 36);
lv_obj_set_x( ui_SettingsExit, -124 );
lv_obj_set_y( ui_SettingsExit, -2 );
lv_obj_set_align( ui_SettingsExit, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_SettingsExit, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_remove_flag( ui_SettingsExit, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_SettingsExit, 30, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_SettingsExit, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_SettingsExit, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_SettingsExit, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_SettingsExit, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_SettingsExit, 2, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SettingsExitLabel = lv_label_create(ui_SettingsExit);
lv_obj_set_width( ui_SettingsExitLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SettingsExitLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_SettingsExitLabel, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_SettingsExitLabel,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_SettingsExitLabel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
lv_label_set_text(ui_SettingsExitLabel,"<");
lv_obj_set_style_text_font(ui_SettingsExitLabel, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SettingsHeadlineLabel = lv_label_create(ui_HeaderSettings);
lv_obj_set_height( ui_SettingsHeadlineLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_flex_grow( ui_SettingsHeadlineLabel, 1);
lv_obj_set_align( ui_SettingsHeadlineLabel, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_SettingsHeadlineLabel,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_SettingsHeadlineLabel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
lv_label_set_text(ui_SettingsHeadlineLabel,"Settings");
lv_obj_set_style_text_align(ui_SettingsHeadlineLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SettingsHeadlineLabel, &lv_font_montserrat_28, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_SettingsHeadlineLabel, -34, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_SettingsHeadlineLabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_SettingsHeadlineLabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_SettingsHeadlineLabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SettingsContents = lv_obj_create(ui_SettingsMain);
lv_obj_remove_style_all(ui_SettingsContents);
lv_obj_set_width( ui_SettingsContents, lv_pct(100));
lv_obj_set_height( ui_SettingsContents, lv_pct(45));
lv_obj_set_x( ui_SettingsContents, -97 );
lv_obj_set_y( ui_SettingsContents, 4 );
lv_obj_set_align( ui_SettingsContents, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_SettingsContents,LV_FLEX_FLOW_ROW_WRAP);
lv_obj_set_flex_align(ui_SettingsContents, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
lv_obj_remove_flag( ui_SettingsContents, LV_OBJ_FLAG_CLICKABLE );    /// Flags
lv_obj_set_scrollbar_mode(ui_SettingsContents, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_scroll_dir(ui_SettingsContents, LV_DIR_VER);
lv_obj_set_style_pad_left(ui_SettingsContents, 8, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_SettingsContents, 8, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_SettingsContents, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_SettingsContents, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_SettingsContents, 8, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_SettingsContents, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_BrightnessPanel = lv_obj_create(ui_SettingsContents);
lv_obj_set_height( ui_BrightnessPanel, 35);
lv_obj_set_width( ui_BrightnessPanel, lv_pct(100));
lv_obj_set_align( ui_BrightnessPanel, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_BrightnessPanel,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_BrightnessPanel, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY);
lv_obj_remove_flag( ui_BrightnessPanel, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_pad_row(ui_BrightnessPanel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_BrightnessPanel, 20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label1 = lv_label_create(ui_BrightnessPanel);
lv_obj_set_width( ui_Label1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label1,"Brightness");

ui_BrightnessSlider = lv_slider_create(ui_BrightnessPanel);
lv_slider_set_range(ui_BrightnessSlider, 10,100);
lv_slider_set_value( ui_BrightnessSlider, 50, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_BrightnessSlider)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_BrightnessSlider, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_BrightnessSlider, 150);
lv_obj_set_height( ui_BrightnessSlider, 10);
lv_obj_set_align( ui_BrightnessSlider, LV_ALIGN_CENTER );


//Compensating for LVGL9.1 draw crash with bar/slider max value when top-padding is nonzero and right-padding is 0
if (lv_obj_get_style_pad_top(ui_BrightnessSlider,LV_PART_MAIN) > 0) lv_obj_set_style_pad_right( ui_BrightnessSlider, lv_obj_get_style_pad_right(ui_BrightnessSlider,LV_PART_MAIN) + 1, LV_PART_MAIN );
ui_EditEnablePanel = lv_obj_create(ui_SettingsContents);
lv_obj_set_height( ui_EditEnablePanel, 35);
lv_obj_set_width( ui_EditEnablePanel, lv_pct(100));
lv_obj_set_align( ui_EditEnablePanel, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_EditEnablePanel,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_EditEnablePanel, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY);
lv_obj_remove_flag( ui_EditEnablePanel, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_pad_row(ui_EditEnablePanel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_EditEnablePanel, 20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label2 = lv_label_create(ui_EditEnablePanel);
lv_obj_set_width( ui_Label2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label2,"Allow Edit");

ui_EditEnable = lv_switch_create(ui_EditEnablePanel);
lv_obj_set_width( ui_EditEnable, 50);
lv_obj_set_height( ui_EditEnable, 25);
lv_obj_set_align( ui_EditEnable, LV_ALIGN_CENTER );
lv_obj_add_state( ui_EditEnable, LV_STATE_CHECKED );     /// States


ui_ScreensaverSettingsPanel = lv_obj_create(ui_SettingsContents);
lv_obj_set_height( ui_ScreensaverSettingsPanel, 35);
lv_obj_set_width( ui_ScreensaverSettingsPanel, lv_pct(100));
lv_obj_set_align( ui_ScreensaverSettingsPanel, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_ScreensaverSettingsPanel,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_ScreensaverSettingsPanel, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY);
lv_obj_remove_flag( ui_ScreensaverSettingsPanel, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_pad_row(ui_ScreensaverSettingsPanel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_ScreensaverSettingsPanel, 20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label3 = lv_label_create(ui_ScreensaverSettingsPanel);
lv_obj_set_width( ui_Label3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label3, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label3,"Screensaver");

ui_ScreensaverTimeout = lv_roller_create(ui_ScreensaverSettingsPanel);
lv_roller_set_options( ui_ScreensaverTimeout, "Off\n5min\n15min\n30min\n60min", LV_ROLLER_MODE_NORMAL );
lv_obj_set_height( ui_ScreensaverTimeout, 100);
lv_obj_set_width( ui_ScreensaverTimeout, LV_SIZE_CONTENT);  /// 1
lv_obj_set_align( ui_ScreensaverTimeout, LV_ALIGN_CENTER );

ui_CounterContainer = lv_obj_create(ui_SettingsMain);
lv_obj_remove_style_all(ui_CounterContainer);
lv_obj_set_width( ui_CounterContainer, lv_pct(100));
lv_obj_set_height( ui_CounterContainer, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_CounterContainer, -69 );
lv_obj_set_y( ui_CounterContainer, 27 );
lv_obj_set_align( ui_CounterContainer, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_CounterContainer,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_CounterContainer, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
lv_obj_remove_flag( ui_CounterContainer, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_pad_left(ui_CounterContainer, 8, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_CounterContainer, 8, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_CounterContainer, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_CounterContainer, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_CounterContainer, 8, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_CounterContainer, 8, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_CounterPanel1 = lv_obj_create(ui_CounterContainer);
lv_obj_set_height( ui_CounterPanel1, 64);
lv_obj_set_flex_grow( ui_CounterPanel1, 1);
lv_obj_set_align( ui_CounterPanel1, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_CounterPanel1,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_CounterPanel1, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
lv_obj_remove_flag( ui_CounterPanel1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_CounterLabel1 = lv_label_create(ui_CounterPanel1);
lv_obj_set_width( ui_CounterLabel1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_CounterLabel1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_CounterLabel1, LV_ALIGN_CENTER );
lv_label_set_text(ui_CounterLabel1,"I");
lv_obj_set_style_text_font(ui_CounterLabel1, &lv_font_montserrat_28, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Counter1 = lv_label_create(ui_CounterPanel1);
lv_obj_set_width( ui_Counter1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Counter1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Counter1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Counter1,"111111");

ui_CounterPanel2 = lv_obj_create(ui_CounterContainer);
lv_obj_set_height( ui_CounterPanel2, 64);
lv_obj_set_flex_grow( ui_CounterPanel2, 1);
lv_obj_set_align( ui_CounterPanel2, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_CounterPanel2,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_CounterPanel2, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
lv_obj_remove_flag( ui_CounterPanel2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_CounterLabel2 = lv_label_create(ui_CounterPanel2);
lv_obj_set_width( ui_CounterLabel2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_CounterLabel2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_CounterLabel2, LV_ALIGN_CENTER );
lv_label_set_text(ui_CounterLabel2,"II");
lv_obj_set_style_text_font(ui_CounterLabel2, &lv_font_montserrat_28, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Counter2 = lv_label_create(ui_CounterPanel2);
lv_obj_set_width( ui_Counter2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Counter2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Counter2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Counter2,"111111");

ui_CounterPanel3 = lv_obj_create(ui_CounterContainer);
lv_obj_set_height( ui_CounterPanel3, 64);
lv_obj_set_flex_grow( ui_CounterPanel3, 1);
lv_obj_set_align( ui_CounterPanel3, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_CounterPanel3,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_CounterPanel3, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
lv_obj_remove_flag( ui_CounterPanel3, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_CounterLabel3 = lv_label_create(ui_CounterPanel3);
lv_obj_set_width( ui_CounterLabel3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_CounterLabel3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_CounterLabel3, LV_ALIGN_CENTER );
lv_label_set_text(ui_CounterLabel3,"III");
lv_obj_set_style_text_font(ui_CounterLabel3, &lv_font_montserrat_28, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Counter3 = lv_label_create(ui_CounterPanel3);
lv_obj_set_width( ui_Counter3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Counter3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Counter3, LV_ALIGN_CENTER );
lv_label_set_text(ui_Counter3,"111111");

ui_CounterPanel4 = lv_obj_create(ui_CounterContainer);
lv_obj_set_height( ui_CounterPanel4, 64);
lv_obj_set_flex_grow( ui_CounterPanel4, 1);
lv_obj_set_align( ui_CounterPanel4, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_CounterPanel4,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_CounterPanel4, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
lv_obj_remove_flag( ui_CounterPanel4, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_CounterLabel4 = lv_label_create(ui_CounterPanel4);
lv_obj_set_width( ui_CounterLabel4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_CounterLabel4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_CounterLabel4, LV_ALIGN_CENTER );
lv_label_set_text(ui_CounterLabel4,"M");
lv_obj_set_style_text_font(ui_CounterLabel4, &lv_font_montserrat_28, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Counter4 = lv_label_create(ui_CounterPanel4);
lv_obj_set_width( ui_Counter4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Counter4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Counter4, LV_ALIGN_CENTER );
lv_label_set_text(ui_Counter4,"111111");

lv_obj_add_event_cb(ui_SettingsExit, ui_event_SettingsExit, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_SettingsHeadlineLabel, ui_event_SettingsHeadlineLabel, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_BrightnessSlider, ui_event_BrightnessSlider, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_EditEnable, ui_event_EditEnable, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_SettingsMain, ui_event_SettingsMain, LV_EVENT_ALL, NULL);

}
