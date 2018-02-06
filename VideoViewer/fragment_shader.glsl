// Begin C++11 raw string literal
R""(

// See http://www.codeproject.com/Articles/236394/Bi-Cubic-and-Bi-Linear-Interpolation-with-GLSL
// See ~/prevproj/2014/filtering/viewer/render.fx

#if !defined(MAKEFILE_DEP)

#if defined(__cplusplus)
#error We are not inside C++ string
#endif

precision highp float; // http://stackoverflow.com/questions/5366416/ and
precision highp int;   // https://github.com/processing/processing/issues/1073 (#if defined(GL_ES))

uniform sampler2D tex;
uniform int kernel_id;
uniform float brightness_term;
uniform float contrast_fac;
uniform float gamma;
uniform float saturation_fac;
uniform vec2 checker_offset;
uniform vec4 through_color;  // use checker if through_color[0]<0.f

in vec2 frag_uv;                // called 'varying' in #version 130
out vec4 frag_color;            // implicitly 'gl_FragColor' in #version 130

float len2(vec2 v) { return dot(v, v); }

vec4 lerp(vec4 v1, vec4 v2, float f) { return (1.f-f)*v1+f*v2; }

// http://stackoverflow.com/questions/24388346/how-to-access-automatic-mipmap-level-in-glsl-fragment-shader-texture
float miplod(vec2 p) {
#if __VERSION__>=400
    return textureQueryLod(tex, p).x; // requires OpenGL 4.00
#endif
// On Intel HD 4000 10.18.10.4276 driver, this use of "#extension" causes shader program validation crash.
#if defined(GL_ARB_texture_query_lod) && 0
#extension GL_ARB_texture_query_lod : require
    return textureQueryLOD(tex, p).x; // extension; note different capitalization than OpenGL 4.00 feature
#endif
    ivec2 texdim = textureSize(tex, 0);
    vec2 ftexdim = vec2(texdim);
    float v2 = max(len2(dFdx(p*ftexdim)), len2(dFdy(p*ftexdim)));
    return log2(v2)*0.5f;
}

// blinear has support [-1, +1]
// blinear[x_] := With[{r = Abs[x]}, If[r < 1, 1-r, 0]]
float blinear(float x) {
    float r = abs(x);
    return (r<1.0f ? 1.0f-r : 0.0f);
}

// keys has support [-2, +2]
// keys[x_] := With[{r = Abs[x]}, 1/2 If[r < 1, 2 + r r (-5 + 3 r), If[r < 2, 4 + r (-8 + (5 - r) r), 0]]]
float keys(float x) {
    float r = abs(x);
    return 0.5f * ( r<1.0f ? 2.0f+r*r*(-5.0f+3.0f*r) :
                    r<2.0f ? 4.0f+r*(-8.0f+(5.0f-r)*r) : 0.0f );
}

// mitchell has support [-2, +2]
// mitchell[x_] := With[{r  = Abs[x]}, 1/18 If[r < 1, 16 + r r (-36 + 21 r),
//   If[r < 2, 32 + r (-60 + (36 - 7 r) r), 0]]];  (* mitchell[x, 1/3, 1/3] *)
float mitchell(float x) {
    float r = abs(x);
    return (1.0f/18.0f) * (r<1.0f ? 16.0f+r*r*(-36.0f+21.0f*r) :
                           r<2.0f ? 32.0f+r*(-60.0f+(36.0f-7.0f*r)*r) : 0.0f);
}

// bspline3 has support [-2, +2]
// bspline3[x_] := With[{r = Abs[x]}, If[r < 1, 2/3 + (-1 + r/2) r^2, If[r <= 2, 4/3 + r (-2 + (1 - r/6) r), 0]]]
float bspline3(float x) {
    float r = abs(x);
    return (r<1.0f ? 2.0f/3.0f+r*r*(-1.0f+r*0.5f) :
            r<2.0f ? 4.0f/3.0f+r*(-2.0f+r*(1.0f-r/6.0f)) : 0.0f);
}

// omoms3 has support [-2, +2]
// omoms3[x_] := With[{r = Abs[x]}, 1/42 If[r < 1, 26 + r (3 + r (-42 + 21 r)),
//                      If[r < 2, 58 + r (-85 + (42 - 7 r) r), 0]]]
float omoms3(float x) {
    float r = abs(x);
    return 1.0f/42.0f*(r<1.0f ? 26.0f+r*(3.0f+r*(-42.0f+r*21.0f)) :
                       r<2.0f ? 58.0f+r*(-85.0f+r*(42.0f+r*-7.0f)) : 0.0f);
}

const float TAU = 6.2831853071795864769f; // N[2 Pi, 20] ;  see http://tauday.com/

float sinc_abs(float x) {
    // Sinc[x]  where here I assume that x>=0
    // ASSERTX(x>=0.);
    return x<1e-9f ? 1.f : sin(x)/x;
}

// lanczos[x_, r_] has support [-r, +r]
// dirichlet[x_, a_] := If[x < -a/2, 0, If[x < a/2, Sinc[Pi x], 0]] // a==W==2*r
// Sinc[x] = Sin[x]/x for x!=0, but is 1 for x=0.
// lanczos[x_, a_] := dirichlet[x, a] Sinc[2 Pi x/a]
// lanczos[x_, r_] := dirichlet[x, 2*r] Sinc[2 Pi x/(2*r)]
float lanczos(float x, float r) {
    x = abs(x);
    return x<r ? sinc_abs((TAU/2.f)*x) * sinc_abs(((TAU/2.f)/r)*x) : 0.f;
}

// lanczos6 has support [-3, +3]
float lanczos6(float x) { return lanczos(x, 3.f); }

// lanczos10 has support [-5, +5]
float lanczos10(float x) { return lanczos(x, 5.f); }

float eval_basis(float x, int basis_id) {
    if (false) { }
    else if (basis_id==1) { return keys(x); }
    else if (basis_id==2) { return lanczos6(x); }
    else if (basis_id==3) { return lanczos10(x); }
    else { return -1.f; }
}

vec4 eval_general(vec2 p, int basis_id, sampler2D ptexture) {
    float l = miplod(p);
    bool is_magnification = l<0.001f;
    // Because I do not manually construct good minified levels, do not use expensive lanczos kernels when minifying.
    if (!is_magnification && basis_id>=2) basis_id = 1;
    // basis support: keys [-2, +2], lanczos6 [-3, +3], lanczos10 [-5, +5]
    int r = basis_id==1 ? 2 : basis_id==2 ? 3 : basis_id==3 ? 5 : -1;
    ivec2 tdim = textureSize(tex, 0);
    vec2 lidim = vec2(tdim);
    vec4 color;
    if (is_magnification) {
        l = 0.f;
        vec4 tcolor = vec4(0.f, 0.f, 0.f, 0.f);
        float tw = 0.f;
        vec2 ps = p*lidim-0.5f; // scaled and offset by 1/2 cell to account for dual sampling of texels
        vec2 pfr = fract(ps); vec2 pfl = ps-pfr;
        vec2 pbase = 1.f-pfr-float(r);
        for (int x = 0; x<2*r; x++) for (int y = 0; y<2*r; y++) {
            float w = eval_basis(pbase[0]+float(x), basis_id) * eval_basis(pbase[1]+float(y), basis_id);
            // tcolor += texture(ptexture, (pfl-float(r)+1.5f+vec2(float(x), float(y)))/lidim) * w;
            tcolor += textureLod(ptexture, (pfl-float(r)+1.5f+vec2(float(x), float(y)))/lidim, 0.f) * w;
            tw += w;
        }
        if (basis_id>=2) tcolor /= tw; // lanczos6 and lanczos10 need renormalization
        // Note that because they are truncated windowed sync approximations, lanczos6 and lanczos10 may
        //   suffer from well known ringing artifacts,
        //   e.g.: VideoViewer ~/data/image/genpattern/rhhhs.bmp -key kk====
        color = tcolor;
    } else {                    // minification
        float lfr = fract(l); float lfl = l-lfr;
        vec4 t0color = vec4(0.f);
        {
            vec2 dim = vec2(textureSize(tex, int(lfl)));
            vec2 ps = p*dim-0.5f;
            vec2 pfr = fract(ps); vec2 pfl = ps-pfr;
            vec2 pbase = 1.f-pfr-float(r);
            float tw = 0.f;
            for (int x = 0; x<2*r; x++) for (int y = 0; y<2*r; y++) {
                float w = eval_basis(pbase[0]+float(x), basis_id) * eval_basis(pbase[1]+float(y), basis_id);
                t0color += textureLod(ptexture, (pfl-float(r)+1.5f+vec2(float(x), float(y)))/dim, lfl+0.f) * w;
                tw += w;
            }
            if (basis_id>=2) t0color /= tw; // lanczos6 and lanczos10 need renormalization
        }
        vec4 t1color = vec4(0.f);
        {
            vec2 dim = vec2(textureSize(tex, int(lfl)+1));
            vec2 ps = p*dim-0.5f;
            vec2 pfr = fract(ps); vec2 pfl = ps-pfr;
            vec2 pbase = 1.f-pfr-float(r);
            float tw = 0.f;
            for (int x = 0; x<2*r; x++) for (int y = 0; y<2*r; y++) {
                float w = eval_basis(pbase[0]+float(x), basis_id) * eval_basis(pbase[1]+float(y), basis_id);
                t1color += textureLod(ptexture, (pfl-float(r)+1.5f+vec2(float(x), float(y)))/dim, lfl+1.f) * w;
                tw += w;
            }
            if (basis_id>=2) t1color /= tw; // lanczos6 and lanczos10 need renormalization
        }
        color = lerp(t0color, t1color, lfr);
    }
    return color;
}

vec4 RGB_to_YUV(vec4 rgb) {
    return vec4(rgb[0]*(vec3(66.f,  -38.f, 112.f)/256.f)+
                rgb[1]*(vec3(129.f, -74.f, -94.f)/256.f)+
                rgb[2]*(vec3(25.f,  112.f, -18.f)/256.f)+
                (vec3(16.f, 128.f, 128.f)/256.f),
                rgb[3]);
}

vec4 YUV_to_RGB(vec4 yuv) {
    return vec4(yuv[0]*(vec3(298.f,  298.f, 298.f)/256.f)+
                yuv[1]*(vec3(0.f,   -100.f, 516.f)/256.f)+
                yuv[2]*(vec3(409.f, -208.f,   0.f)/256.f)+
                (vec3(-298.f*16.f-409.f*128.f, -298.f*16.f+100.f*128.f+208.f*128.f, -298.f*16.f-516.f*128.f)/65536.f),
                yuv[3]);
}

void main() {
    // gl_FragColor = vec4(gl_TexCoord[0].st, 0.f, 1.f); return;
    vec4 color;
    if (kernel_id==0 || kernel_id==4) { // linear, nearest
        color = texture(tex, frag_uv);
    } else {
        color = eval_general(frag_uv, kernel_id, tex);
    }
    vec4 backcolor = through_color;
    if (backcolor[0]<0.f) {                   // use checkerboard pattern; e.g.: vv ~/data/image/dancer_charts.png
        backcolor = vec4(1.f, 1.f, 1.f, 1.f); // white
        vec4 coord = gl_FragCoord;            // center of the lower-left pixel is (0.5, 0.5)
        int checker_size = 6;
        // Bad mod at left and bottom (where coordinates cross into negative):
        // ivec2 coord2 = ivec2(floor(coord.xy+checker_offset))/checker_size;
        // int evenodd = (coord2.x+coord2.y)%2;
        // Correct mod even for negative numbers:
        vec2 coord2 = floor((coord.xy+checker_offset)/checker_size);
        float evenodd = mod(coord2.x+coord2.y, 2.f);
        backcolor = vec4(backcolor.rgb*evenodd, 1.f);
    }
    if (true) {                 // OVER background color
        color = color + (1.f-color.a)*backcolor;
    }
    if (false) color = vec4(1.f, 1.f, 0.f, 1.f); // yellow
    if (brightness_term!=0.f || contrast_fac!=1.f || gamma!=1.f || saturation_fac!=1.f) {
        // http://www.poynton.com/notes/brightness_and_contrast/
        vec4 yuv = RGB_to_YUV(color);
        float y = yuv[0];
        y = pow(y, gamma);
        y *= contrast_fac;
        y += brightness_term;
        yuv[0] = y;
        for (int c = 1; c<3; c++) yuv[c] = .5f+(yuv[c]-.5f)*saturation_fac;
        color = YUV_to_RGB(yuv);
    }
    frag_color = color;
}

#endif // !defined(MAKEFILE_DEP)

)""                             // end C++11 raw string literal
