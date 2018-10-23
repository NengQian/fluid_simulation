#pragma once

namespace merely3d
{
    namespace shaders
    {
        
const char * const basic_fragment = R"~~~(
#version 330 core

uniform vec3 object_color;

out vec4 FragColor;

void main()
{
    FragColor = vec4(object_color, 1.0);
}
)~~~";

const char * const basic_vertex = R"~~~(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}

)~~~";

const char * const default_fragment = R"~~~(
#version 330 core

in vec3 normal_world;
in vec3 frag_pos_world;
in vec3 frag_pos_local;

uniform vec3 light_color;
uniform vec3 object_color;

// The position of the camera in world coordinates
uniform vec3 view_pos;

// Light direction is direction from light source to fragment (in world coordinates)
uniform vec3 light_dir;

// The transform taking the reference shape (i.e. a unit cube) into
// the actual shape of the object (i.e. a box with certain extents)
uniform mat3 reference_transform;

uniform float pattern_grid_size;

out vec4 FragColor;

void main()
{
    // TODO: Make ambient/specular etc. configurable
    float ambient_strength = 0.15;
    float specular_strength = 0.5;

    // frag_pos_local gives us local coordinates in the reference
    // primitive (i.e. unit cube). We need to transform by the reference
    // transform in order to obtain the actual local coordinates of the
    // logical entity (i.e. a box with certain extents)
    vec3 local_pos = reference_transform * frag_pos_local;

    // Assign the fragment to a grid cell and determine if the grid cell should
    // be patterned
    ivec3 grid_coords = pattern_grid_size > 0.0
        ? ivec3(round(local_pos / pattern_grid_size))
        : ivec3(0);
    bool patterned = (grid_coords[0] + grid_coords[1] + grid_coords[2]) % 2 != 0;

    vec3 base_color = patterned
                    ? 0.9 * object_color
                    : object_color;

    vec3 normal = normalize(normal_world);

    // Ambient
    vec3 ambient = ambient_strength * light_color;

    // Diffuse
    float diff = max(- dot(normal, light_dir), 0.0);
    vec3 diffuse = diff * light_color;

    // Specular
    vec3 view_dir = normalize(frag_pos_world - view_pos);
    vec3 reflect_dir = reflect(light_dir, normal);
    float spec = pow(max(- dot(view_dir, reflect_dir), 0.0), 32);
    vec3 specular = specular_strength * spec * light_color;

    vec3 result = (ambient + diffuse + specular) * base_color;
    FragColor = vec4(result, 1.0);
}

)~~~";

const char * const default_vertex = R"~~~(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 normal_world;
out vec3 frag_pos_world;
out vec3 frag_pos_local;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform mat3 normal_transform;

void main()
{
    vec4 world_pos = model * vec4(aPos, 1.0);
    normal_world = normalize(normal_transform * aNormal);
    frag_pos_world = vec3(world_pos);
    frag_pos_local = aPos;
    gl_Position = projection * (view * world_pos);
}

)~~~";

const char * const particle_fragment = R"~~~(
#version 330 core

in VertexData
{
    vec3 frag_color;
    vec3 sphere_pos_view;
    float sphere_radius;
} vs_in;

uniform mat4 projection;
uniform mat4 inv_projection;
uniform vec3 light_color;
uniform vec3 light_dir_eye;
uniform float viewport_width;
uniform float viewport_height;
uniform float near_plane_dist;

out vec4 FragColor;

/// Computes the direction vector (in camera/eye space) of the ray
/// associated with the current fragment
vec3 compute_ray_direction()
{
    float ndc_x = 2.0 * (gl_FragCoord.x - viewport_width / 2.0) / viewport_width;
    float ndc_y = 2.0 * (gl_FragCoord.y - viewport_height / 2.0) / viewport_height;
    vec4 ndc_point = vec4(ndc_x, ndc_y, -1.0, 1.0);
    vec4 view_point = inv_projection * near_plane_dist * ndc_point;
    return vec3(view_point);
}

/// Performs an intersection test between a ray and a sphere,
/// returning the parameter t such that `t * ray_direction`
/// gives the point of intersection if any such point exists.
///
/// Note that if there is no intersection, a negative value of
/// t is returned, which means that one cannot distinguish between
/// the case when there is no intersection and the case in which
/// the ray starts within the sphere. This is simply because we wish
/// to discard the entire sphere if the ray starts from inside of the sphere.
float intersect_ray_sphere(vec3 ray_direction, vec3 sphere_center, float radius)
{
    // d = the direction vector of the ray
    // c = sphere center
    // r = radius * radius
    float dTd = dot(ray_direction, ray_direction);
    float cTc = dot(sphere_center, sphere_center);
    float dTc = dot(ray_direction, sphere_center);
    float r2 = radius * radius;

    // points on the ray satisfy X = t * d for some t >= 0.
    // points on the sphere satisfy || X - c || = r^2
    // points in the intersection satisfy
    // t^2 d^d - 2 t d^c + c^c - r^2 = 0     (*)
    //
    // We seek a non-negative t for which (*) is satisfied. If no such t exists,
    // there is no intersection. In this particular case, we signal this by returning
    // a *negative* t. The solution is given by *real* solutions to the quadratic formula.
    // If only complex solutions exist, there is no intersection.

    // Cancelling terms in the quadratic equation, we have that
    //  t = [ d^T c +- sqrt( (d^T c)^2 - d^T d * (c^T c - r^2) ) ] / (d^T d)
    // we refer to the term in the square root as the discriminant.

    float discriminant = dTc * dTc - dTd * (cTc - r2);

    if (discriminant < 0)
    {
        return -1.0;
    }
    else
    {
        float sqrt_disc = sqrt(discriminant);
        float t1 = (dTc + sqrt_disc) / dTd;
        float t2 = (dTc - sqrt_disc) / dTd;
        return min(t1, t2);
    }
}

void main()
{
    vec3 ray = compute_ray_direction();
    float t = intersect_ray_sphere(ray, vs_in.sphere_pos_view, vs_in.sphere_radius);

    if (t >= 0)
    {
        // The point x on the sphere (surface)
        vec3 x = t * ray;

        // Compute correct depth for fragment
        vec4 clip = projection * vec4(x, 1.0);
        float ndc_z = clip.z / clip.w;
        float window_depth = 0.5 * (ndc_z + 1.0);
        gl_FragDepth = gl_DepthRange.diff * window_depth + gl_DepthRange.near;

        vec3 normal = normalize(x - vs_in.sphere_pos_view);

        // Ambient
        // TODO: Make strength configurable
        float ambient_strength = 0.15;
        vec3 ambient = ambient_strength * light_color;

        // Diffuse
        float diff = max(- dot(normal, light_dir_eye), 0.0);
        vec3 diffuse = diff * light_color;

        // Specular
        // TODO: Make strength configurable
        float specular_strength = 0.5;
        vec3 view_dir = normalize(x);
        vec3 reflect_dir = reflect(light_dir_eye, normal);
        float spec = pow(max(- dot(view_dir, reflect_dir), 0.0), 16);
        vec3 specular = specular_strength * spec * light_color;

        vec3 result = (ambient + diffuse + specular) * vs_in.frag_color;

        FragColor = vec4(result, 1.0);
    }
    else
    {
        discard;
    }
}

)~~~";

const char * const particle_geometry = R"~~~(
#version 330 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in VertexData
{
    vec3 sphere_color;
    float sphere_radius;
} vs_in[];

out VertexData
{
    vec3 frag_color;
    vec3 sphere_pos_view;
    float sphere_radius;
} vs_out;

uniform mat4 projection;

vec3 orthogonal_to_view_vector(vec3 v)
{
    // We assume here that v.z != 0!
    return vec3(v.y, -v.x, 0);
}

void main() {
    // Input point is the center of the sphere in view space
    vec3 c = vec3(gl_in[0].gl_Position);

    float r = vs_in[0].sphere_radius;

    // TODO: Need to assert positive radius elsewhere in the C++ code
    if (r < -c.z)
    {
        // Determine the part of the sphere which is at all visible
        // from the current vantage point. This can be determined by
        // fitting a cone starting at the view center (origin in view space)
        // such that it just exactly encloses the sphere, in which case
        // the intersection of the cone and the sphere is a circle
        // of radius w, situated at a distance d along the vector
        // pointing to the sphere's center. We next construct
        // a billboard which is just big enough to fit this disk inside.
        float g = r / c.z;
        float w = r * sqrt(1 - g * g);
        float d = -c.z * w * w / (r * r);

        vec3 billboard_center = d * normalize(c);

        // Construct unit vectors p and q which span the plane which is orthogonal
        // to the vector pointing to x
        vec3 p = normalize(orthogonal_to_view_vector(c));
        vec3 q = normalize(cross(c, p));

        // TODO: Currently we just make the billboard much bigger than necessary,
        // so that we know that it's big enough. Need to adapt this so that
        // we create one that is only just big enough (to avoid having to discard
        // too many fragments)
        vec3 offsets[4];
        offsets[0] = w * (- p - q);
        offsets[1] = w * (- p + q);
        offsets[2] = w * (+ p - q);
        offsets[3] = w * (+ p + q);

        for (int i = 0; i < 4; ++i)
        {
            gl_Position = projection * vec4(billboard_center + offsets[i], 1.0);
            vs_out.frag_color = vs_in[0].sphere_color;
            vs_out.sphere_radius = vs_in[0].sphere_radius;
            vs_out.sphere_pos_view = c;
            EmitVertex();
        }
    }

    EndPrimitive();
}

)~~~";

const char * const particle_vertex = R"~~~(
#version 330 core
layout (location = 0) in vec3 pos;
layout (location = 1) in vec3 color;
layout (location = 2) in float radius;

out VertexData
{
    vec3 sphere_color;
    float sphere_radius;
} vs_out;

uniform mat4 projection;
uniform mat4 view;

void main()
{
    vec4 view_pos = view * vec4(pos, 1.0);
    vs_out.sphere_radius = radius;
    vs_out.sphere_color = color;
    gl_Position = view_pos;
}

)~~~";

    }
}
