
#pragma once

#include <merely3d/frame.hpp>
#include <iostream>
#include <fstream>
#include <string.h>
#include "imgui/imgui.h"
#include "math_types.hpp"


using merely3d::renderable;
using merely3d::Rectangle;
using merely3d::Box;
using merely3d::Line;
using merely3d::Color;
using merely3d::Material;
using merely3d::Sphere;
using merely3d::red;
using merely3d::Particle;

using Eigen::AngleAxisf;
using Eigen::Vector3f;
using namespace Eigen;
using Simulator::RealVector3;

    class marching_cube_visual
    {
    public:

        marching_cube_visual(){};

        void render(merely3d::Frame &frame)
        {
            // Below we demonstrate how to render various primitives with merely3d.

            frame.draw(renderable(Rectangle(0.5, 0.5))
                               .with_position(1.0, 0.0, 0.5)
                               .with_orientation(AngleAxisf(0.78, Vector3f(1.0f, 0.0f, 0.0f)))
                               .with_material(Material().with_color(Color(0.5, 0.3, 0.3))));

            frame.draw(renderable(Box(1.0, 1.0, 1.0))
                               .with_position(4.0, 0.0, 1.1));

            frame.draw(renderable(Box(0.2, 5.0, 1.0))
                               .with_position(0.0, 0.0, 1.0));

            frame.draw(renderable(Box(0.2, 1.0, 1.0))
                               .with_position(0.0, 0.0, 5.0)
                               .with_orientation(AngleAxisf(0.5, Vector3f(1.0, 1.0, 1.0)))
                               .with_material(Material().with_color(red())));

            const auto floor_color = Color(0.5f, 0.35f, 0.35f);
            frame.draw(renderable(Rectangle(20.0f, 20.0f))
                               .with_position(0.0f, 0.0f, 0.0f)
                               .with_material(Material().with_color(floor_color)));

            frame.draw(renderable(Sphere(1.0))
                               .with_position(3.0, 3.0, 3.0));

            // Note that if you use RealVector3 for your position, you might need to cast it into a Vector3f
            // since merely3d uses float for positions. Here's an example:
            const RealVector3 my_position(1.0, 5.0, 10.0);
            frame.draw(renderable(Sphere(3.0))
                        .with_position(my_position.cast<float>()));

            frame.draw_line(Line(Vector3f(0.0, 0.0, 0.0), Vector3f(10.0, -5.0, 10.0)));
            /// Begin begins a new ImGui window that you can move around as you please
            if (ImGui::Begin("Hello", NULL, ImVec2(300, 200)))
            {


            }
            ImGui::End();


        }
    };



