"""Shared Chinese tuning-desk typography and colors."""
def setup_gui(desk):
    from imgui_bundle import imgui
    def fonts():
        from .presentation import _font
        path = _font(18).path
        io = imgui.get_io()
        glyphs = [0x20, 0xFF, 0x2000, 0x206F, 0x3000, 0x30FF, 0x4E00, 0x9FFF, 0xFF00, 0xFFEF, 0]
        io.font_default = io.fonts.add_font_from_file_ttf(path, 18, glyph_ranges_as_int_list=glyphs)
        desk.heading_font = io.fonts.add_font_from_file_ttf(path, 25, glyph_ranges_as_int_list=glyphs)
    def style():
        imgui.style_colors_light()
        s = imgui.get_style(); s.window_padding = imgui.ImVec2(16, 12); s.item_spacing = imgui.ImVec2(10, 8)
        s.frame_padding = imgui.ImVec2(8, 5); s.frame_rounding = 3; s.child_rounding = 4
        for name, color in dict(window_bg=(.94,.96,.98,1), child_bg=(.99,.995,1,1),
            text=(.10,.18,.28,1), text_disabled=(.34,.40,.47,1), border=(.77,.82,.87,1),
            button=(.80,.88,.96,1), button_hovered=(.65,.79,.94,1), button_active=(.48,.68,.89,1),
            check_mark=(.12,.40,.69,1), header=(.75,.85,.94,1), plot_lines=(.12,.40,.69,1)).items():
            s.set_color_(getattr(imgui.Col_, name), imgui.ImVec4(*color))
    fonts();style()

