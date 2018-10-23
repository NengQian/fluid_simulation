#pragma once

#include <merely3d/events.hpp>

#include <memory>

namespace Simulator
{
    class ImGuiEventHandler : public merely3d::EventHandler
    {
    public:
        ImGuiEventHandler();

        ~ImGuiEventHandler() override;

        bool key_press(merely3d::Window &window,
                       merely3d::Key key,
                       merely3d::Action action,
                       int scancode,
                       int modifiers) override;

        void before_frame(merely3d::Window &window, merely3d::Frame & frame) override;

        void after_frame(merely3d::Window &window, double frame_duration) override;

        bool mouse_button_press(merely3d::Window &window, merely3d::MouseButton button, merely3d::Action action,
                                int modifiers) override;

        bool character_input(merely3d::Window &window, unsigned int codepoint) override;

        bool scroll(merely3d::Window &window, double xoffset, double yoffset) override;

    private:
        class Data;

        Data * _d;
    };
}
