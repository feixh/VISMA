// http://coding-experiments.blogspot.com/2010/06/edge-detection.html
#version 430 core

in vec2 uv;
//out vec3 color;
out float color;
uniform sampler2D this_texture;
uniform float z_near = 0.05;
uniform float z_far = 5.0;

float rows;
float cols;

const float kThreshLow = 0.05;
const float kThreshHigh = 0.10;


/// \brief Decide whether the given pixel is an edge pixel.
/// \param thresh_low: If value lower than this one, NON-EDGE pixel.
/// \param thresh_high: If value higher than this one, EDGE pixel.
/// Otherwise, return the actual value.
float threshold(in float thresh_low, in float thresh_high , in float value) {
    if (value < thresh_low) { return 0.0; }
    if (value >= thresh_high) { return 1.0; }
    return (value - thresh_low) / (thresh_high-thresh_low);
}

// averaged pixel intensity from 3 color channels
float mean_intensity(in vec4 pix) {
    return (pix.r + pix.g + pix.b)/3.;
}

float linearize_depth(in float z) {
    if (z == 1) return -1;
    return 2 * z_near * z_far / (z_far + z_near - (2 * z - 1) * (z_far - z_near));
}

float is_edge(in vec2 pos){
    vec2 size = textureSize(this_texture, 0);
    float dx = 1.0 / size.x;  // step length along x axis
    float dy = 1.0 / size.y; // step length along y axis

    if (pos.x < 5 * dx || pos.x > 1 - 5 * dx
    || pos.y < 5 * dy || pos.y > 1 - 5 * dy) {
        return 0;
    }
    float value[9]; // pixel values
    float delta;

    value[0] = linearize_depth(texture(this_texture, pos+vec2(-dx,-dy)).r);
    value[1] = linearize_depth(texture(this_texture, pos+vec2(-dx,  0)).r);
    value[2] = linearize_depth(texture(this_texture, pos+vec2(-dx,+dy)).r);
    value[3] = linearize_depth(texture(this_texture, pos+vec2(0,  -dy)).r);
    value[4] = linearize_depth(texture(this_texture, pos+vec2(0,    0)).r);
    value[5] = linearize_depth(texture(this_texture, pos+vec2(0,  +dy)).r);
    value[6] = linearize_depth(texture(this_texture, pos+vec2(+dx,-dy)).r);
    value[7] = linearize_depth(texture(this_texture, pos+vec2(+dx,  0)).r);
    value[8] = linearize_depth(texture(this_texture, pos+vec2(+dx,+dy)).r);

    if (value[4] == -1) return 0;

    // Average color differences around neighboring values.
    delta = 0.25*(abs(value[1]-value[7]) + abs(value[5]-value[3]) + abs(value[0]-value[8]) + abs(value[2]-value[6]));
//    float ddy = 3*value[0]  - 3*value[2] + 10*value[3] - 10*value[5] + 3*value[6] - 3*value[8];
//    float ddx = 3*value[0]  + 10*value[1] + 3*value[2] - 3*value[6] - 10*value[7] - 3*value[8];
//    delta = 0.5*(abs(ddx) + abs(ddy));

    // In the case of depth texture, pixel values have actual meanings: they are depth in metric space.
    // Thus the thresholds are also in metric space. (0.001, 0.05) seems fine.
//    return threshold(kThreshLow, kThreshHigh, delta);

    // However, depth map are non-linear by nature: closeby pixels have finer depth values (since there are denser samples)
    // while far pixels have very coarse depth values (since there are fewer samples).
    // We can linearize the depth and then the thresholds change accordingly.
    return threshold(kThreshLow, kThreshHigh, delta);
}

void main()
{
    color = is_edge(vec2(uv.x, uv.y));
}
