unsigned int im_current_color = 0xffffffff;
float im_current_thickness = 1.0f;

void ImColor4f(float r, float g, float b, float a)
{
    int ri = r*255.0f; if (ri < 0) ri = 0; if (ri > 255) ri = 255;
    int gi = g*255.0f; if (gi < 0) gi = 0; if (gi > 255) gi = 255;
    int bi = b*255.0f; if (bi < 0) bi = 0; if (bi > 255) bi = 255;
    int ai = a*255.0f; if (ai < 0) ai = 0; if (ai > 255) ai = 255;
    im_current_color = (ai << 24) | (bi << 16) | (gi << 8) | ri;
}

void ImColor4f(vdb_color c)
{
    ImColor4f(c.r, c.g, c.b, c.a);
}

void ImLineWidth(float thickness)
{
    im_current_thickness = thickness;
}

void ImFillCircle(float x, float y, float r)
{
    using namespace ImGui;
    float x_win, y_win;
    vdbModelToWindow(x, y, 0, 1, &x_win, &y_win);
    GetWindowDrawList()->AddCircleFilled(ImVec2(x_win, y_win), r, im_current_color);
}

void ImFillRect(float x1, float y1, float x2, float y2)
{
    using namespace ImGui;
    float x1_win, y1_win, x2_win, y2_win;
    vdbModelToWindow(x1, y1, 0, 1, &x1_win, &y1_win);
    vdbModelToWindow(x2, y2, 0, 1, &x2_win, &y2_win);
    GetWindowDrawList()->AddRectFilled(ImVec2(x1_win, y1_win), ImVec2(x2_win, y2_win), im_current_color);
}

void ImLine(float x1, float y1, float x2, float y2)
{
    using namespace ImGui;
    float x1_win, y1_win, x2_win, y2_win;
    vdbModelToWindow(x1, y1, 0, 1, &x1_win, &y1_win);
    vdbModelToWindow(x2, y2, 0, 1, &x2_win, &y2_win);
    GetWindowDrawList()->AddLine(ImVec2(x1_win, y1_win), ImVec2(x2_win, y2_win), im_current_color, im_current_thickness);
}

void ImDrawRect(float x1, float y1, float x2, float y2)
{
    ImLine(x1, y1, x2, y1);
    ImLine(x2, y1, x2, y2);
    ImLine(x2, y2, x1, y2);
    ImLine(x1, y2, x1, y1);
}

void ImBegin()
{
    using namespace ImGui;
    SetNextWindowPos(ImVec2(0,0));
    SetNextWindowSize(ImVec2(vdbWindowWidth(), vdbWindowHeight()));
    PushStyleColor(ImGuiCol_WindowBg, ImVec4(0,0,0,0));
    Begin("##vdb_imgui_custom_draw", NULL, ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoSavedSettings|ImGuiWindowFlags_NoInputs|ImGuiWindowFlags_NoFocusOnAppearing|ImGuiWindowFlags_NoBringToFrontOnFocus);
}

void ImEnd()
{
    using namespace ImGui;
    End();
    PopStyleColor();
}
