// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.3
// LVGL version: 9.1.0
// Project name: design

#include "ui.h"

_ui_local_style_t* _ui_local_styles;
uint32_t _ui_local_style_count = 0;


inline void ui_object_set_local_style_property
(lv_obj_t* object_p, lv_style_selector_t selector, lv_style_prop_t property, ui_style_variable_t value ) {
    if ( object_p!=NULL && lv_obj_is_valid(object_p) ) {
        lv_obj_set_local_style_prop( object_p, property, _ui_style_value_convert( property, value ), selector );
    }
}

//Atomic function to set (and register) an LVGL local style-property for a given part+state (selector) of an object (widget) - can be used in many ways due to being atomic
void ui_object_set_themeable_style_property
(lv_obj_t* object_p, lv_style_selector_t selector, lv_style_prop_t property, const ui_theme_variable_t* theme_variable_p) {
    static _ui_local_style_t* local_style_p;
    static _ui_local_style_property_setting_t* property_setting_p;

    if (object_p==NULL || !lv_obj_is_valid(object_p) || theme_variable_p==NULL) return;
    local_style_p = _ui_local_style_create( theme_variable_p, true );
    if (local_style_p == NULL) return;
    property_setting_p = _ui_local_style_property_setting_create( local_style_p, object_p, selector, property );
    if (property_setting_p == NULL) return;

    lv_obj_set_local_style_prop( object_p, property, _ui_style_value_convert( property, ui_get_theme_value(theme_variable_p) ), selector ); //ui_object_set_local_style_property( object_p, selector, property, ui_get_theme_value(theme_variable_p) );
}


//This function goes through all registered style-property settings and if theme is changed, sets all of them to the theme. (If called periodically, it can follow change of values automatically.)
void _ui_theme_set_variable_styles (uint8_t mode) {
    static uint8_t ui_theme_idx_previous = -1;
    static uint32_t i, j;
    static _ui_local_style_property_setting_t* property_setting_p;
    static ui_style_variable_t  style_value;
    static ui_style_variable_t* style_variable_p;

    uint8_t ui_Theme_Changed = (ui_theme_idx != ui_theme_idx_previous); ui_theme_idx_previous = ui_theme_idx;

    for (i = 0; i < _ui_local_style_count; ++i) {

        style_variable_p = _ui_local_styles[i].style_variable_p;
        if (_ui_local_styles[i].is_themeable) style_value = ui_get_theme_value( style_variable_p );
        else style_value = *style_variable_p;

        if (style_variable_p != _ui_local_styles[i].previous_pointer || style_value != _ui_local_styles[i].previous_value
            || mode == UI_VARIABLE_STYLES_MODE_INIT || ui_Theme_Changed) {
            _ui_local_styles[i].previous_pointer = style_variable_p; _ui_local_styles[i].previous_value = style_value;

            property_setting_p = _ui_local_styles[i].style_property_settings; //first item of linked list
            for (j = 0; j < _ui_local_styles[i].style_property_setting_count; ++j) {
                if (property_setting_p->object_p != NULL) {
                    ui_object_set_local_style_property (
                     property_setting_p->object_p, property_setting_p->selector, property_setting_p->property, style_value
                    );
                }
                if (property_setting_p->next_p != NULL) property_setting_p = (_ui_local_style_property_setting_t*) property_setting_p->next_p; //get next item in linked-list for next round
                else break; //this shouldn't happen, but if it does, it's assumed end of the list
            }
        }
    }
}


ui_style_variable_t ui_get_theme_value(const ui_theme_variable_t* var) {
    return var[ui_theme_idx];
}


lv_style_value_t _ui_style_value_convert (lv_style_prop_t property, ui_style_variable_t value) {
    static lv_style_value_t Style_Value; //LVGL would produce artefacts if both .num and .color were set for the local style to add:
    //static lv_style_const_prop_t ValueConvert_Table [_LV_STYLE_NUM_BUILT_IN_PROPS] = { [LV_STYLE_BG_COLOR] = LV_STYLE_CONST_BG_COLOR(1), [LV_STYLE_BG_OPA] = LV_STYLE_CONST_BG_OPA(1) };
    if (property==LV_STYLE_BG_COLOR || property==LV_STYLE_BG_GRAD_COLOR || property==LV_STYLE_BG_IMAGE_RECOLOR || property==LV_STYLE_BORDER_COLOR
        || property==LV_STYLE_OUTLINE_COLOR || property==LV_STYLE_SHADOW_COLOR || property==LV_STYLE_IMAGE_RECOLOR || property==LV_STYLE_LINE_COLOR
        || property==LV_STYLE_ARC_COLOR || property==LV_STYLE_TEXT_COLOR) {
        Style_Value.color = lv_color_hex(value);
    }
    else if (property==LV_STYLE_BG_GRAD || property==LV_STYLE_BG_IMAGE_SRC || property==LV_STYLE_ARC_IMAGE_SRC || property==LV_STYLE_TEXT_FONT
             || property==LV_STYLE_COLOR_FILTER_DSC || property==LV_STYLE_ANIM || property==LV_STYLE_TRANSITION || property==LV_STYLE_BITMAP_MASK_SRC) {
        Style_Value.ptr = (void*)(uintptr_t) value;
    }
    else Style_Value.num = value;
    return Style_Value;
}


//auto-update dynamic local style array with existing/new style-variable (1st dimension)
_ui_local_style_t* _ui_local_style_create (const ui_style_variable_t* style_variable_p, bool is_themeable) {
    static uint32_t i;
    static _ui_local_style_t* local_style_p;

    for (i = 0; i < _ui_local_style_count; ++i) { //Find existing local style
        if (_ui_local_styles[i].style_variable_p == style_variable_p) return &_ui_local_styles[i];
    }
    //If not found, create new local style
    _ui_local_styles = (_ui_local_style_t*) lv_realloc( _ui_local_styles, (_ui_local_style_count + 1) * sizeof(_ui_local_style_t) );
    LV_ASSERT_MALLOC( _ui_local_styles );
    if (_ui_local_styles == NULL) return NULL;
    //Reset new local style
    local_style_p = &_ui_local_styles[ _ui_local_style_count ];
    local_style_p->style_variable_p = (ui_style_variable_t*) style_variable_p;
    local_style_p->is_themeable = is_themeable;
    local_style_p->previous_pointer = NULL;
    local_style_p->previous_value = -1;
    local_style_p->style_property_setting_count = 0;
    local_style_p->style_property_settings = NULL;
    #ifdef LV_SQUARELINE_THEME__EXPLICIT_GARBAGE_COLLECTOR_PERIOD
    local_style_p->garbage_collector_couter = 0;
    #endif

    ++_ui_local_style_count;
    return local_style_p;
}

//auto-update dynamic local style-array's 2nd dimension (object part+state style-property settings for a given style-variable)
_ui_local_style_property_setting_t* _ui_local_style_property_setting_create
(_ui_local_style_t* local_style_p, lv_obj_t* object_p, lv_style_selector_t selector, lv_style_prop_t property) {
    static uint32_t i; //auto-update 
    static _ui_local_style_property_setting_t *style_property_setting_p, *empty_style_property_setting_p;

    #ifdef LV_SQUARELINE_THEME__EXPLICIT_GARBAGE_COLLECTOR_PERIOD
    style_property_setting_p = local_style_p->style_property_settings; //first item of linked list
    if (style_property_setting_p != NULL) { //checking might be unnecessary if count is properly initialized to 0
        if (local_style_p->garbage_collector_couter > 0) --local_style_p->garbage_collector_couter; //avoid calling garbage collector every time
        else if (LV_SQUARELINE_THEME__EXPLICIT_GARBAGE_COLLECTOR_PERIOD > 0) {
            local_style_p->garbage_collector_couter = LV_SQUARELINE_THEME__EXPLICIT_GARBAGE_COLLECTOR_PERIOD - 1;
            for (i = 0; i < local_style_p->style_property_setting_count; ++i) {
                if (style_property_setting_p->object_p != NULL) { //check only valid object entry (skip already emptied positions{
                    #if ( defined(LV_SQUARELINE_THEME__OBJECT_VALIDITY_CACHE) && LV_SQUARELINE_THEME__OBJECT_VALIDITY_CACHE )  //Not recommended inside explicit garbage collector.
                    if ( style_property_setting_p->validcheck_counter > 0 ) --style_property_setting_p->validcheck_counter; //avoids calling complex lv_obj_is_valid() many times
                    else { //check objects with expired positive vaidity check
                        if ( lv_obj_is_valid( style_property_setting_p->object_p ) ) style_property_setting_p->validcheck_counter = LV_SQUARELINE_THEME__OBJECT_VALIDITY_CACHE;
                        else style_property_setting_p->object_p = NULL; //free up an entry of a deleted widget
                    }
                    #else  //this is the recommended variant inside explicit garbage collector:
                    if ( !lv_obj_is_valid( style_property_setting_p->object_p ) ) style_property_setting_p->object_p = NULL; //free up an entry of a deleted widget
                    #endif
                }
                if (style_property_setting_p->next_p != NULL) style_property_setting_p = (_ui_local_style_property_setting_t*) style_property_setting_p->next_p; //get next item in linked-list for next round
                else break; //this shouldn't happen
            }
        }
    }
    #endif
    style_property_setting_p = local_style_p->style_property_settings; //first item of linked list
    empty_style_property_setting_p = NULL;
    if (style_property_setting_p != NULL) { //checking might be unnecessary if count is properly initialized to 0
        for (i = 0; i < local_style_p->style_property_setting_count; ++i) {
            if (empty_style_property_setting_p == NULL) { //search and register one empty position that might be needed for a newly created entry
                if (style_property_setting_p->object_p == NULL) empty_style_property_setting_p = style_property_setting_p; //if found empty position, register it
                #ifdef LV_SQUARELINE_THEME__IMPLICIT_GARBAGE_COLLECTOR
                else { //if not, check for possible empty positions of deleted objects (garbage-collection)
                    #if ( defined(LV_SQUARELINE_THEME__OBJECT_VALIDITY_CACHE) && LV_SQUARELINE_THEME__OBJECT_VALIDITY_CACHE )  //for slow hardware, adding this and setting to 5..10 in lv_conf.h might make it faster
                    if ( style_property_setting_p->validcheck_counter > 0 ) --style_property_setting_p->validcheck_counter; //avoids calling complex lv_obj_is_valid() many times
                    else { //check objects with expired positive vaidity check
                        if ( lv_obj_is_valid( style_property_setting_p->object_p ) ) style_property_setting_p->validcheck_counter = LV_SQUARELINE_THEME__OBJECT_VALIDITY_CACHE;
                        else { //free up an entry of a deleted widget
                            style_property_setting_p->object_p = NULL;
                            empty_style_property_setting_p = style_property_setting_p;
                        }
                    }
                    #else
                    if ( !lv_obj_is_valid( style_property_setting_p->object_p ) ) { //free up an entry of a deleted widget
                        style_property_setting_p->object_p = NULL;
                        empty_style_property_setting_p = style_property_setting_p;
                    }
                    #endif
                }
                #endif
            }
            if (style_property_setting_p->object_p == object_p && style_property_setting_p->selector == selector
                && style_property_setting_p->property == property) { //setting found in the list (created already), so returning it
                return style_property_setting_p;
            }
            if (style_property_setting_p->next_p != NULL) style_property_setting_p = (_ui_local_style_property_setting_t*) style_property_setting_p->next_p; //get next item in linked-list for next round
            else break; //this shouldn't happen, but if it does, create a new element
        }
    }
    //If not found, create new local style-property (can be inside the array at freed-up places of deleted objects too)
    if (empty_style_property_setting_p == NULL) { //allocate new
        empty_style_property_setting_p = (_ui_local_style_property_setting_t*) lv_malloc( sizeof(_ui_local_style_property_setting_t) );
        LV_ASSERT_MALLOC( empty_style_property_setting_p );
        if (empty_style_property_setting_p == NULL) return NULL;
        if (style_property_setting_p != NULL) style_property_setting_p->next_p = (void*) empty_style_property_setting_p; //create link from last item to the new
        else local_style_p->style_property_settings = empty_style_property_setting_p; //(except if creating first item)
        style_property_setting_p = empty_style_property_setting_p; //take the new item pointer for initialization
        style_property_setting_p->next_p = NULL; //signify the end of the linked list, just in case
        ++local_style_p->style_property_setting_count;
    }
    else { //reuse empty
        style_property_setting_p = empty_style_property_setting_p;
    }
    style_property_setting_p->object_p = object_p;
    style_property_setting_p->selector = selector;
    style_property_setting_p->property = property;
    #ifdef LV_SQUARELINE_THEME__OBJECT_VALIDITY_CACHE
    style_property_setting_p->validcheck_counter = LV_SQUARELINE_THEME__OBJECT_VALIDITY_CACHE; //0
    #endif
    return style_property_setting_p;
}

