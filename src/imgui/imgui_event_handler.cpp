/**
 * This file mostly copies functionality from the GLFW example in the Dear Imgui repository,
 * making necessary adjustment to fit well with the abstractions provided by merely3d.
 */

#include "imgui_event_handler.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <merely3d/window.hpp>

#include <iostream>

namespace Simulator
{
    struct MouseButtonsState
    {
        bool left_down;
        bool right_down;
        bool middle_down;

        // We need these additional flags in case press and release come directly after each other
        bool left_just_pressed;
        bool right_just_pressed;
        bool middle_just_pressed;

        MouseButtonsState()
                : left_down(false),
                  right_down(false),
                  middle_down(false),
                  left_just_pressed(false),
                  right_just_pressed(false),
                  middle_just_pressed(false)
        {}
    };

    // TODO: Currently we unable to properly destroy/cleanup OpenGL textures, buffers and et cetera,
    // because we have no guarantee that the correct context is active and even valid
    // at the time of destruction. Can we fix this somehow?
    struct ImGuiEventHandler::Data
    {
        typedef void(*ImGuiContextDestroyFunc)(ImGuiContext *);
        std::unique_ptr<ImGuiContext, ImGuiContextDestroyFunc> context;

        bool initialized;

        Data()
            : context(ImGui::CreateContext(), ImGui::DestroyContext),
              initialized(false),
              shader_program(0),
              attrib_location_tex(0),
              attrib_location_proj_mtx(0),
              attrib_location_position(0),
              attrib_location_uv(0),
              attrib_location_color(0),
              vbo_handle(0),
              elements_handle(0)
        {

        }

        GLuint shader_program;
        GLuint vertex_shader_handle;
        GLuint fragment_shader_handle;

        GLint attrib_location_tex;
        GLint attrib_location_proj_mtx;
        GLint attrib_location_position;
        GLint attrib_location_uv;
        GLint attrib_location_color;

        GLuint vbo_handle;
        GLuint elements_handle;
        GLuint fonts_texture;

        MouseButtonsState mouse_buttons_state;

        void init(merely3d::Window & window);
        void create_device_objects(merely3d::Window & window);
        void create_fonts_texture();
        void create_keyboard_mapping();
        void render();
    };

    void ImGuiEventHandler::Data::create_device_objects(merely3d::Window &window)
    {
        // Backup GL state
        GLint last_texture, last_array_buffer, last_vertex_array;
        glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
        glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &last_array_buffer);
        glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vertex_array);

        const GLchar* vertex_shader =
                "#version 330 core\n"
                "uniform mat4 ProjMtx;\n"
                "in vec2 Position;\n"
                "in vec2 UV;\n"
                "in vec4 Color;\n"
                "out vec2 Frag_UV;\n"
                "out vec4 Frag_Color;\n"
                "void main()\n"
                "{\n"
                "	Frag_UV = UV;\n"
                "	Frag_Color = Color;\n"
                "	gl_Position = ProjMtx * vec4(Position.xy,0,1);\n"
                "}\n";

        const GLchar* fragment_shader =
                "#version 330 core\n"
                "uniform sampler2D Texture;\n"
                "in vec2 Frag_UV;\n"
                "in vec4 Frag_Color;\n"
                "out vec4 Out_Color;\n"
                "void main()\n"
                "{\n"
                "	Out_Color = Frag_Color * texture( Texture, Frag_UV.st);\n"
                "}\n";

        shader_program = glCreateProgram();
        vertex_shader_handle = glCreateShader(GL_VERTEX_SHADER);
        fragment_shader_handle = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(vertex_shader_handle, 1, &vertex_shader, NULL);
        glShaderSource(fragment_shader_handle, 1, &fragment_shader, NULL);
        glCompileShader(vertex_shader_handle);
        glCompileShader(fragment_shader_handle);
        glAttachShader(shader_program, vertex_shader_handle);
        glAttachShader(shader_program, fragment_shader_handle);
        glLinkProgram(shader_program);

        const auto check_shader_compile_error = [&] (GLuint shader_id)
        {
            int  success;
            glGetShaderiv(shader_id, GL_COMPILE_STATUS, &success);

            if (!success)
            {
                char info_log[512];
                glGetShaderInfoLog(shader_id, 512, NULL, info_log);
                std::string error_msg(info_log);
                throw std::runtime_error("Shader compilation error: " + error_msg);
            }
        };

        check_shader_compile_error(vertex_shader_handle);
        check_shader_compile_error(fragment_shader_handle);

        int success;
        glGetProgramiv(shader_program, GL_LINK_STATUS, &success);
        if (!success)
        {
            char info_log[512];
            glGetProgramInfoLog(shader_program, 512, NULL, info_log);
            std::string error_msg(info_log);
            throw std::runtime_error("Program link error: " + error_msg);
        }

        attrib_location_tex = glGetUniformLocation(shader_program, "Texture");
        attrib_location_proj_mtx = glGetUniformLocation(shader_program, "ProjMtx");
        attrib_location_position = glGetAttribLocation(shader_program, "Position");
        attrib_location_uv = glGetAttribLocation(shader_program, "UV");
        attrib_location_color = glGetAttribLocation(shader_program, "Color");

        glGenBuffers(1, &vbo_handle);
        glGenBuffers(1, &elements_handle);

        create_fonts_texture();

        // Restore modified GL state
        glBindTexture(GL_TEXTURE_2D, last_texture);
        glBindBuffer(GL_ARRAY_BUFFER, last_array_buffer);
        glBindVertexArray(last_vertex_array);
    }

    void ImGuiEventHandler::Data::init(merely3d::Window &window)
    {
        ImGui::SetCurrentContext(context.get());
        ImGuiIO& io = ImGui::GetIO();

        create_device_objects(window);
        create_keyboard_mapping();
        initialized = true;
    }

    void ImGuiEventHandler::Data::create_fonts_texture()
    {
        // Build texture atlas
        ImGuiIO& io = ImGui::GetIO();
        unsigned char* pixels;
        int width, height;
        io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

        // Upload texture to graphics system
        GLint last_texture;
        glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
        glGenTextures(1, &fonts_texture);
        glBindTexture(GL_TEXTURE_2D, fonts_texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

        // Store our identifier
        io.Fonts->TexID = (void *)(intptr_t) fonts_texture;

        // Restore state
        glBindTexture(GL_TEXTURE_2D, last_texture);
    }

    void ImGuiEventHandler::Data::render()
    {
        // Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
        ImGuiIO& io = ImGui::GetIO();
        int fb_width = (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
        int fb_height = (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
        if (fb_width == 0 || fb_height == 0)
            return;

        const auto draw_data = ImGui::GetDrawData();
        draw_data->ScaleClipRects(io.DisplayFramebufferScale);

        // Backup GL state
        GLenum last_active_texture; glGetIntegerv(GL_ACTIVE_TEXTURE, (GLint*)&last_active_texture);
        glActiveTexture(GL_TEXTURE0);
        GLint last_program; glGetIntegerv(GL_CURRENT_PROGRAM, &last_program);
        GLint last_texture; glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
        GLint last_sampler; glGetIntegerv(GL_SAMPLER_BINDING, &last_sampler);
        GLint last_array_buffer; glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &last_array_buffer);
        GLint last_element_array_buffer; glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &last_element_array_buffer);
        GLint last_vertex_array; glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vertex_array);
        GLint last_polygon_mode[2]; glGetIntegerv(GL_POLYGON_MODE, last_polygon_mode);
        GLint last_viewport[4]; glGetIntegerv(GL_VIEWPORT, last_viewport);
        GLint last_scissor_box[4]; glGetIntegerv(GL_SCISSOR_BOX, last_scissor_box);
        GLenum last_blend_src_rgb; glGetIntegerv(GL_BLEND_SRC_RGB, (GLint*)&last_blend_src_rgb);
        GLenum last_blend_dst_rgb; glGetIntegerv(GL_BLEND_DST_RGB, (GLint*)&last_blend_dst_rgb);
        GLenum last_blend_src_alpha; glGetIntegerv(GL_BLEND_SRC_ALPHA, (GLint*)&last_blend_src_alpha);
        GLenum last_blend_dst_alpha; glGetIntegerv(GL_BLEND_DST_ALPHA, (GLint*)&last_blend_dst_alpha);
        GLenum last_blend_equation_rgb; glGetIntegerv(GL_BLEND_EQUATION_RGB, (GLint*)&last_blend_equation_rgb);
        GLenum last_blend_equation_alpha; glGetIntegerv(GL_BLEND_EQUATION_ALPHA, (GLint*)&last_blend_equation_alpha);
        GLboolean last_enable_blend = glIsEnabled(GL_BLEND);
        GLboolean last_enable_cull_face = glIsEnabled(GL_CULL_FACE);
        GLboolean last_enable_depth_test = glIsEnabled(GL_DEPTH_TEST);
        GLboolean last_enable_scissor_test = glIsEnabled(GL_SCISSOR_TEST);

        // Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, polygon fill
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_CULL_FACE);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_SCISSOR_TEST);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Setup viewport, orthographic projection matrix
        glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
        const float ortho_projection[4][4] =
                {
                        { 2.0f/io.DisplaySize.x, 0.0f,                   0.0f, 0.0f },
                        { 0.0f,                  2.0f/-io.DisplaySize.y, 0.0f, 0.0f },
                        { 0.0f,                  0.0f,                  -1.0f, 0.0f },
                        {-1.0f,                  1.0f,                   0.0f, 1.0f },
                };
        glUseProgram(shader_program);
        glUniform1i(attrib_location_tex, 0);
        glUniformMatrix4fv(attrib_location_proj_mtx, 1, GL_FALSE, &ortho_projection[0][0]);
        glBindSampler(0, 0); // Rely on combined texture/sampler state.

        // Recreate the VAO every time
        // (This is to easily allow multiple GL contexts. VAO are not shared among GL contexts,
        // and we don't track creation/deletion of windows so we don't have an obvious key to use to cache them.)
        GLuint vao_handle = 0;
        glGenVertexArrays(1, &vao_handle);
        glBindVertexArray(vao_handle);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_handle);
        glEnableVertexAttribArray(attrib_location_position);
        glEnableVertexAttribArray(attrib_location_uv);
        glEnableVertexAttribArray(attrib_location_color);
        glVertexAttribPointer(attrib_location_position, 2, GL_FLOAT, GL_FALSE, sizeof(ImDrawVert), (GLvoid*)IM_OFFSETOF(ImDrawVert, pos));
        glVertexAttribPointer(attrib_location_uv, 2, GL_FLOAT, GL_FALSE, sizeof(ImDrawVert), (GLvoid*)IM_OFFSETOF(ImDrawVert, uv));
        glVertexAttribPointer(attrib_location_color, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(ImDrawVert), (GLvoid*)IM_OFFSETOF(ImDrawVert, col));

        // Draw
        for (int n = 0; n < draw_data->CmdListsCount; n++)
        {
            const ImDrawList* cmd_list = draw_data->CmdLists[n];
            const ImDrawIdx* idx_buffer_offset = 0;

            glBindBuffer(GL_ARRAY_BUFFER, vbo_handle);
            glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)cmd_list->VtxBuffer.Size * sizeof(ImDrawVert), (const GLvoid*)cmd_list->VtxBuffer.Data, GL_STREAM_DRAW);

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements_handle);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)cmd_list->IdxBuffer.Size * sizeof(ImDrawIdx), (const GLvoid*)cmd_list->IdxBuffer.Data, GL_STREAM_DRAW);

            for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++)
            {
                const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
                if (pcmd->UserCallback)
                {
                    pcmd->UserCallback(cmd_list, pcmd);
                }
                else
                {
                    glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->TextureId);
                    glScissor((int)pcmd->ClipRect.x, (int)(fb_height - pcmd->ClipRect.w), (int)(pcmd->ClipRect.z - pcmd->ClipRect.x), (int)(pcmd->ClipRect.w - pcmd->ClipRect.y));
                    glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount, sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer_offset);
                }
                idx_buffer_offset += pcmd->ElemCount;
            }
        }
        glDeleteVertexArrays(1, &vao_handle);

        // Restore modified GL state
        glUseProgram(last_program);
        glBindTexture(GL_TEXTURE_2D, last_texture);
        glBindSampler(0, last_sampler);
        glActiveTexture(last_active_texture);
        glBindVertexArray(last_vertex_array);
        glBindBuffer(GL_ARRAY_BUFFER, last_array_buffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, last_element_array_buffer);
        glBlendEquationSeparate(last_blend_equation_rgb, last_blend_equation_alpha);
        glBlendFuncSeparate(last_blend_src_rgb, last_blend_dst_rgb, last_blend_src_alpha, last_blend_dst_alpha);
        if (last_enable_blend) glEnable(GL_BLEND); else glDisable(GL_BLEND);
        if (last_enable_cull_face) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
        if (last_enable_depth_test) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
        if (last_enable_scissor_test) glEnable(GL_SCISSOR_TEST); else glDisable(GL_SCISSOR_TEST);
        glPolygonMode(GL_FRONT_AND_BACK, (GLenum)last_polygon_mode[0]);
        glViewport(last_viewport[0], last_viewport[1], (GLsizei)last_viewport[2], (GLsizei)last_viewport[3]);
        glScissor(last_scissor_box[0], last_scissor_box[1], (GLsizei)last_scissor_box[2], (GLsizei)last_scissor_box[3]);
    }

    void ImGuiEventHandler::Data::create_keyboard_mapping()
    {
        using merely3d::Key;

        auto & io = ImGui::GetIO();
        // TODO: More keys...?
        io.KeyMap[ImGuiKey_Tab] = static_cast<int>(Key::Tab);
        io.KeyMap[ImGuiKey_LeftArrow] = static_cast<int>(Key::Left);
        io.KeyMap[ImGuiKey_RightArrow] = static_cast<int>(Key::Right);
        io.KeyMap[ImGuiKey_UpArrow] = static_cast<int>(Key::Up);
        io.KeyMap[ImGuiKey_DownArrow] = static_cast<int>(Key::Down);
        io.KeyMap[ImGuiKey_PageUp] = static_cast<int>(Key::PageUp);
        io.KeyMap[ImGuiKey_PageDown] = static_cast<int>(Key::PageDown);
        io.KeyMap[ImGuiKey_Home] = static_cast<int>(Key::Home);
        io.KeyMap[ImGuiKey_End] = static_cast<int>(Key::End);
        io.KeyMap[ImGuiKey_Insert] = static_cast<int>(Key::Insert);
        io.KeyMap[ImGuiKey_Delete] = static_cast<int>(Key::Delete);
        io.KeyMap[ImGuiKey_Backspace] = static_cast<int>(Key::Backspace);
        io.KeyMap[ImGuiKey_Space] = static_cast<int>(Key::Space);
        io.KeyMap[ImGuiKey_Enter] = static_cast<int>(Key::Enter);
        io.KeyMap[ImGuiKey_Escape] = static_cast<int>(Key::Escape);
        io.KeyMap[ImGuiKey_A] = static_cast<int>(Key::A);
        io.KeyMap[ImGuiKey_C] = static_cast<int>(Key::C);
        io.KeyMap[ImGuiKey_V] = static_cast<int>(Key::V);
        io.KeyMap[ImGuiKey_X] = static_cast<int>(Key::X);
        io.KeyMap[ImGuiKey_Y] = static_cast<int>(Key::Y);
        io.KeyMap[ImGuiKey_Z] = static_cast<int>(Key::Z);
    }

    ImGuiEventHandler::ImGuiEventHandler()
    {
        _d = new ImGuiEventHandler::Data();
    }

    ImGuiEventHandler::~ImGuiEventHandler()
    {
        delete _d;
    }

    void Simulator::ImGuiEventHandler::before_frame(merely3d::Window &window, merely3d::Frame & frame)
    {
        if (!_d->initialized)
        {
            _d->init(window);
        }

        ImGuiIO& io = ImGui::GetIO();

        auto glfw_window = window.glfw_window();

        // Setup display size (every frame to accommodate for window resizing)
        int w, h;
        int display_w, display_h;
        glfwGetWindowSize(glfw_window, &w, &h);
        glfwGetFramebufferSize(glfw_window, &display_w, &display_h);
        io.DisplaySize = ImVec2((float)w, (float)h);
        io.DisplayFramebufferScale = ImVec2(w > 0 ? ((float)display_w / w) : 0, h > 0 ? ((float)display_h / h) : 0);

        // Setup time step
        io.DeltaTime = static_cast<float>(frame.time_since_prev_frame());

        // Setup inputs
        // (we already got mouse wheel, keyboard keys & characters from glfw callbacks polled in glfwPollEvents())
        if (glfwGetWindowAttrib(glfw_window, GLFW_FOCUSED))
        {
            // Set OS mouse position if requested (only used when ImGuiConfigFlags_NavEnableSetMousePos is enabled by user)
            if (io.WantSetMousePos)
            {
                glfwSetCursorPos(glfw_window, (double)io.MousePos.x, (double)io.MousePos.y);
            }
            else
            {
                double mouse_x, mouse_y;
                glfwGetCursorPos(glfw_window, &mouse_x, &mouse_y);
                io.MousePos = ImVec2((float)mouse_x, (float)mouse_y);
            }
        }
        else
        {
            io.MousePos = ImVec2(-FLT_MAX,-FLT_MAX);
        }

        auto & state = _d->mouse_buttons_state;

        io.MouseDown[0] = state.left_just_pressed || state.left_down;
        io.MouseDown[1] = state.right_just_pressed || state.right_down;
        io.MouseDown[2] = state.middle_just_pressed || state.middle_down;

        state.left_just_pressed = false;
        state.right_just_pressed = false;
        state.middle_just_pressed = false;

        ImGui::NewFrame();
    }

    void Simulator::ImGuiEventHandler::after_frame(merely3d::Window &window, double frame_duration)
    {
        int display_w, display_h;
        glfwGetFramebufferSize(window.glfw_window(), &display_w, &display_h);
        ImGui::Render();
        _d->render();
    }

    bool ImGuiEventHandler::mouse_button_press(merely3d::Window &window, merely3d::MouseButton button,
                                               merely3d::Action action, int modifiers)
    {
        using merely3d::MouseButton;
        using merely3d::Action;

        const auto pressed = action == Action::Press || action == Action::Repeat;
        const auto released = action == Action::Release;

        auto & state = _d->mouse_buttons_state;

        switch (button)
        {
            case MouseButton::Left:
                state.left_down = pressed;
                state.left_just_pressed = state.left_just_pressed || released;
                break;
            case MouseButton::Right:
                state.right_down = pressed;
                state.right_just_pressed = state.left_just_pressed || released;
                break;
            case MouseButton::Middle:
                state.middle_down = pressed;
                state.middle_just_pressed = state.left_just_pressed || released;
                break;
            case MouseButton::Other:
                break;
        }

        const auto & io = ImGui::GetIO();

        // Stop propagation if ImGui wants to capture the input
        return io.WantCaptureMouse;
    }

    bool ImGuiEventHandler::key_press(merely3d::Window &window,
                                      merely3d::Key key,
                                      merely3d::Action action,
                                      int scancode,
                                      int modifiers)
    {
        using merely3d::Action;
        using merely3d::Key;
        auto & io = ImGui::GetIO();

        const auto is_down = action == Action::Press || action == Action::Repeat;
        io.KeysDown[static_cast<int>(key)] = is_down;

        // According to the GLFW example for Dear ImGui,
        // modifiers are not completely reliable, so we explicitly check instead
        const auto lctrl = static_cast<int>(Key::LeftControl);
        const auto rctrl = static_cast<int>(Key::RightControl);
        const auto lalt = static_cast<int>(Key::LeftAlt);
        const auto ralt = static_cast<int>(Key::RightAlt);
        const auto lshift = static_cast<int>(Key::LeftShift);
        const auto rshift = static_cast<int>(Key::RightShift);
        const auto lsuper = static_cast<int>(Key::LeftSuper);
        const auto rsuper = static_cast<int>(Key::RightSuper);

        io.KeyCtrl = io.KeysDown[lctrl] || io.KeysDown[rctrl];
        io.KeyShift = io.KeysDown[lshift] || io.KeysDown[rshift];
        io.KeyAlt = io.KeysDown[lalt] || io.KeysDown[ralt];
        io.KeySuper = io.KeysDown[lsuper] || io.KeysDown[rsuper];

        return io.WantCaptureKeyboard;
    }

    bool ImGuiEventHandler::character_input(merely3d::Window &window, unsigned int codepoint)
    {
        auto & io = ImGui::GetIO();

        if (codepoint >= 0 && codepoint < 0x10000)
        {
            io.AddInputCharacter(static_cast<unsigned short>(codepoint));
        }

        return io.WantTextInput;
    }

    bool ImGuiEventHandler::scroll(merely3d::Window &window, double xoffset, double yoffset)
    {
        auto & io = ImGui::GetIO();
        io.MouseWheelH += static_cast<float>(xoffset);
        io.MouseWheel += static_cast<float>(yoffset);

        return io.WantCaptureMouse;
    }

}
