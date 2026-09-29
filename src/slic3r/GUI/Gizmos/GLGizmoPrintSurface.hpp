#ifndef slic3r_GLGizmoPrintSurface_hpp_
#define slic3r_GLGizmoPrintSurface_hpp_

#include "GLGizmoPainterBase.hpp"

#include "slic3r/GUI/I18N.hpp"

// Orca: paints the faces a part is printed onto in non-planar (S4) layers offset from them; the print surface
// is generated under or inside them (print surface "Painted faces").

namespace Slic3r::GUI {

class GLGizmoPrintSurface : public GLGizmoPainterBase
{
public:
    GLGizmoPrintSurface(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id);

    void render_painter_gizmo() override;

protected:
    void        on_render_input_window(float x, float y, float bottom_limit) override;
    std::string on_get_name() const override;

    void render_tooltip_button(float x, float y);

    wxString handle_snapshot_action_name(bool shift_down, Button button_down) const override;

    std::string get_gizmo_entering_text() const override { return _u8L("Entering Paint-on print surface"); }
    std::string get_gizmo_leaving_text() const override { return _u8L("Leaving Paint-on print surface"); }
    std::string get_action_snapshot_name() const override { return _u8L("Paint-on print surface editing"); }

    EnforcerBlockerType get_left_button_state_type() const override { return EnforcerBlockerType::ENFORCER; }
    EnforcerBlockerType get_right_button_state_type() const override { return EnforcerBlockerType::NONE; }

    // BBS
    wchar_t                           m_current_tool = 0;

private:
    bool on_init() override;

    void update_model_object() override;
    void update_from_model_object(bool first_update) override;

    void             on_opening() override {}
    void             on_shutdown() override;
    PainterGizmoType get_painter_type() const override;

    // This map holds all translated description texts, so they can be easily referenced during layout calculations
    // etc. When language changes, GUI is recreated, and this class constructed again, so the change takes effect.
    std::map<std::string, wxString> m_desc;

    // Contains all shortcuts in the format of {shortcut, description}, e.g. {alt + _L("Left mouse button"), _L("Part_selection")}
    std::vector<std::pair<wxString, wxString>> m_shortcuts_brush;
    // Contains all shortcuts in the format of {shortcut, description}, e.g. {alt + _L("Left mouse button"), _L("Part_selection")}
    std::vector<std::pair<wxString, wxString>> m_shortcuts_triangle;
    // Contains all shortcuts in the format of {shortcut, description}, e.g. {alt + _L("Left mouse button"), _L("Part_selection")}
    std::vector<std::pair<wxString, wxString>> m_shortcuts_smart_fill;
};

} // namespace Slic3r::GUI

#endif // slic3r_GLGizmoPrintSurface_hpp_
