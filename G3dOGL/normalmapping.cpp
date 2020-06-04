// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include <memory>  // _WIN32 bug: must appear before <mutex> to avoid warning 4548
#include <mutex>   // std::once_flag, std::call_once()

#include "HW.h"
#include "libHh/Array.h"
#include "libHh/Image.h"
#include "libHh/StringOp.h"
#include "normalmapping.h"

// (setenv NORMAL_MAPPING ogl2; G3dOGL ~/data/mesh/buddhaf.nf10000.m -key Dt)
//  also frag1 nvrc dot3

namespace hh {

static Vec4<float> normalizef(const Vec3<float>& v) { return concat(v * .5f + .5f, V(0.f)); }

struct NormalMapping_ogl2 final : NormalMapping {
  using type = NormalMapping_ogl2;
  // Great tutorial: http://zach.in.tu-clausthal.de/teaching/cg_literatur/glsl_tutorial/
  //   "GLSL Tutorial von Lighthouse3D"
  string name() const override { return "ogl2"; }
  bool is_supported() const override { return assertx(glGetString(GL_VERSION))[0] >= '2'; }
  void init() override {
    USE_GL_EXT(glCreateShader, PFNGLCREATESHADERPROC);
    USE_GL_EXT(glShaderSource, PFNGLSHADERSOURCEPROC);
    USE_GL_EXT(glCompileShader, PFNGLCOMPILESHADERPROC);
    USE_GL_EXT(glGetShaderiv, PFNGLGETSHADERIVPROC);
    USE_GL_EXT(glGetShaderInfoLog, PFNGLGETSHADERINFOLOGPROC);
    USE_GL_EXT(glCreateProgram, PFNGLCREATEPROGRAMPROC);
    USE_GL_EXT(glAttachShader, PFNGLATTACHSHADERPROC);
    USE_GL_EXT(glLinkProgram, PFNGLLINKPROGRAMPROC);
    USE_GL_EXT(glGetProgramiv, PFNGLGETPROGRAMIVPROC);
    USE_GL_EXT(glGetProgramInfoLog, PFNGLGETPROGRAMINFOLOGPROC);
    USE_GL_EXT(glValidateProgram, PFNGLVALIDATEPROGRAMPROC);
    USE_GL_EXT(glIsProgram, PFNGLISPROGRAMPROC);
    fragment_shader_id = assertx(glCreateShader(GL_FRAGMENT_SHADER));
    const GLint len = narrow_cast<GLint>(fragment_shader.size());
    const char* pprogram = fragment_shader.c_str();
    glShaderSource(fragment_shader_id, 1, &pprogram, &len);
    glCompileShader(fragment_shader_id);
    {
      GLint params;
      glGetShaderiv(fragment_shader_id, GL_COMPILE_STATUS, &params);
      if (params != GL_TRUE) {
        GLint slen;
        glGetShaderiv(fragment_shader_id, GL_INFO_LOG_LENGTH, &slen);
        Array<char> str(slen, '\0');
        glGetShaderInfoLog(fragment_shader_id, str.num(), nullptr, str.data());
        SHOWL;
        SHOW(str.data());
        _exit(1);
      }
    }
    program_id = assertx(glCreateProgram());
    glAttachShader(program_id, fragment_shader_id);
    // glBindFragDataLocation(program_id, 0, "gl_FragColor");
    glLinkProgram(program_id);
    {
      GLint params;
      glGetProgramiv(program_id, GL_LINK_STATUS, &params);
      if (params != GL_TRUE) {
        GLint slen;
        glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &slen);
        Array<char> str(slen, '\0');
        glGetProgramInfoLog(program_id, str.num(), nullptr, str.data());
        SHOWL;
        SHOW(str.data());
        _exit(1);
      }
    }
    glValidateProgram(program_id);
    {
      GLint params;
      glGetProgramiv(program_id, GL_VALIDATE_STATUS, &params);
      assertx(params == GL_TRUE);
    }
    assertx(glIsProgram(program_id));
    assertx(!gl_report_errors());
  }
  void set_parameters(const Vector& lightdirmodel, const Vector& eyedirmodel, float ambient, float lightsource,
                      const Pixel& meshcolor_s) override {
    Vector vhalf = ok_normalized(lightdirmodel + eyedirmodel);
    Vector scaled_light = lightdirmodel * lightsource;
    // just pull the Red channel out; (default is gray 0.5f)
    float meshspecular = meshcolor_s[0] / 255.f;
    assertw(meshcolor_s[0] == meshcolor_s[1]);
    assertw(meshcolor_s[0] == meshcolor_s[2]);
    const int phong = 4;
    Vector scaled_vhalf = vhalf * pow(lightsource * meshspecular, 1.f / phong);
    USE_GL_EXT(glUseProgram, PFNGLUSEPROGRAMPROC);
    USE_GL_EXT(glGetUniformLocation, PFNGLGETUNIFORMLOCATIONPROC);
    USE_GL_EXT(glUniform3fv, PFNGLUNIFORM3FVPROC);
    USE_GL_EXT(glUniform1fv, PFNGLUNIFORM1FVPROC);
    glUseProgram(program_id);
    GLint loc_vlight = glGetUniformLocation(program_id, "vlight");
    assertx(loc_vlight >= 0);
    GLint loc_vhalf = glGetUniformLocation(program_id, "vhalf");
    assertx(loc_vhalf >= 0);
    GLint loc_ambient = glGetUniformLocation(program_id, "ambient");
    assertx(loc_ambient >= 0);
    glUniform3fv(loc_vlight, 1, scaled_light.data());
    glUniform3fv(loc_vhalf, 1, scaled_vhalf.data());
    glUniform1fv(loc_ambient, 1, V(ambient).data());
    assertx(!gl_report_errors());
  }
  void activate() override {
    USE_GL_EXT(glUseProgram, PFNGLUSEPROGRAMPROC);
    glUseProgram(program_id);
    assertx(!gl_report_errors());
  }
  void deactivate() override {
    USE_GL_EXT(glUseProgram, PFNGLUSEPROGRAMPROC);
    glUseProgram(0);  // go back to fixed-function pipeline; see https://www.opengl.org/sdk/docs/man2/
    // and http://stackoverflow.com/questions/13546461/what-does-gluseprogram0-do
    assertx(!gl_report_errors());
    // glDetachShader(program_id, fragment_shader_id);
    // glDeleteShader(fragment_shader_id);
    // glDeleteProgram(program_id);
  }
  //
  GLuint program_id;
  GLuint fragment_shader_id;
  const string unused_vertex_shader = &R"(
        void main() {
            gl_TexCoord[0] = gl_MultiTexCoord0;
            gl_Position = ftransform();
        }
    )"[1];
  const string fragment_shader = &R"(
        uniform vec3 vlight;
        uniform vec3 vhalf;
        uniform float ambient;

        uniform sampler2D tex;

        varying vec3 tc;                // texture coordinates
        // varying vec4 color;             // incoming fragment primary color

        void main() {
            // gl_FragColor = vec4(gl_TexCoord[0].st, 0., 1.); return;
            vec3 vnorm = texture(tex, gl_TexCoord[0].st).xyz * 2. - 1.;  // texture.stpq
            vnorm = normalize(vnorm);
            if (0) { gl_FragColor = vec4(vnorm.xyz, 1.); return; }
            float vdot = max(0., dot(vnorm, vhalf));
            float adjustment1 = 1.2, adjustment2 = 1.4;
            gl_FragColor = (gl_FrontMaterial.diffuse * (ambient + adjustment1 * max(0., dot(vnorm, vlight))) +
                            gl_FrontMaterial.specular * adjustment2 * pow(vdot, 4.));
            gl_FragColor.a = 1.;
        }
    )"[1];
  static type& instance() {
    static type& f = *new type;
    return f;
  }
};

struct NormalMapping_frag1 final : NormalMapping {
  using type = NormalMapping_frag1;
  string name() const override { return "frag1"; }
  bool is_supported() const override { return contains(gl_extensions_string(), "GL_ARB_fragment_program"); }
  void init() override {
    USE_GL_EXT(glGenProgramsARB, PFNGLGENPROGRAMSARBPROC);
    USE_GL_EXT(glBindProgramARB, PFNGLBINDPROGRAMARBPROC);
    USE_GL_EXT(glProgramStringARB, PFNGLPROGRAMSTRINGARBPROC);
    USE_GL_EXT(glIsProgramARB, PFNGLISPROGRAMARBPROC);
    glGenProgramsARB(1, &program_id);
    assertx(program_id);
    // Setup the program string
    GLuint len = narrow_cast<int>(fragment_shader.size());
    glBindProgramARB(GL_FRAGMENT_PROGRAM_ARB, program_id);
    glProgramStringARB(GL_FRAGMENT_PROGRAM_ARB, GL_PROGRAM_FORMAT_ASCII_ARB, len,
                       reinterpret_cast<const GLubyte*>(fragment_shader.c_str()));
    if (glGetError() != GL_NO_ERROR) {
      GLint pos;
      glGetIntegerv(GL_PROGRAM_ERROR_POSITION_ARB, &pos);
      const char* serr = reinterpret_cast<const char*>(glGetString(GL_PROGRAM_ERROR_STRING_ARB));
      showf("Shader error: %s\n", serr);
      showf("Error loading the program at location %d: %.30s\n", pos, fragment_shader.c_str() + pos);
      assertnever("Error parsing program string");
    }
    assertx(glIsProgramARB(program_id));
    assertx(!gl_report_errors());
  }
  void set_parameters(const Vector& lightdirmodel, const Vector& eyedirmodel, float ambient, float lightsource,
                      const Pixel& meshcolor_s) override {
    Vector vhalf = ok_normalized(lightdirmodel + eyedirmodel);
    Vector scaled_light = lightdirmodel * lightsource;
    // just pull the Red channel out; (default is gray 0.5f)
    float meshspecular = meshcolor_s[0] / 255.f;
    assertw(meshcolor_s[0] == meshcolor_s[1]);
    assertw(meshcolor_s[0] == meshcolor_s[2]);
    const int phong = 4;
    Vector scaled_vhalf = vhalf * pow(lightsource * meshspecular, 1.f / phong);
    USE_GL_EXT(glBindProgramARB, PFNGLBINDPROGRAMARBPROC);
    USE_GL_EXT(glProgramLocalParameter4fvARB, PFNGLPROGRAMLOCALPARAMETER4FVARBPROC);
    glBindProgramARB(GL_FRAGMENT_PROGRAM_ARB, program_id);
    // C0 = light_vector, scalar_ambient
    Vec4<float> light2 = normalizef(scaled_light);
    light2[3] = ambient;
    // C1 = vhalf_vector, 0.f
    Vec4<float> vhalf2 = normalizef(scaled_vhalf);
    glProgramLocalParameter4fvARB(GL_FRAGMENT_PROGRAM_ARB, 0, light2.data());
    glProgramLocalParameter4fvARB(GL_FRAGMENT_PROGRAM_ARB, 1, vhalf2.data());
    assertx(!gl_report_errors());
  }
  void activate() override {
    USE_GL_EXT(glBindProgramARB, PFNGLBINDPROGRAMARBPROC);
    glEnable(GL_FRAGMENT_PROGRAM_ARB);
    glBindProgramARB(GL_FRAGMENT_PROGRAM_ARB, program_id);
    assertx(!gl_report_errors());
  }
  void deactivate() override {
    glDisable(GL_FRAGMENT_PROGRAM_ARB);
    // glDeleteProgramsARB(1, &program_id);
    assertx(!gl_report_errors());
  }
  //
  GLuint program_id;
  // Unfortunately, C++ preprocessor does not like the '#' comment character as first non-whitespace.
  // # OPTION ARB_precision_hint_fastest;
  // # PARAM  vhalf = { 0.5, 0.5, 0.5, 0.5 };
  // # OUTPUT oCol = result.color;
  // http://en.wikipedia.org/wiki/ARB_assembly_language
  // https://www.opengl.org/registry/specs/ARB/fragment_program.txt
  const string fragment_shader = R"(!!ARBfp1.0
        TEMP    norm, light, vhalf, ndotl, ndoth, diffuse;
        ATTRIB  norm_tc = fragment.texcoord[0];
        ATTRIB  color = fragment.color.primary;
        PARAM   lightraw = program.local[0];
        PARAM   vhalfraw = program.local[1];

        TEX     norm.xyz, norm_tc, texture[0], 2D;
        MAD     norm.xyz, norm, {2.0, 2.0, 2.0}, {-1.0, -1.0, -1.0};

        DP3     norm.a, norm, norm;
        RSQ     norm.a, norm.a;
        MUL     norm.xyz, norm, norm.a;

        MAD     vhalf.xyz, vhalfraw, {2.0, 2.0, 2.0}, {-1.0, -1.0, -1.0};
        DP3_SAT ndoth.a, norm, vhalf;
        MUL     ndoth.a, ndoth.a, ndoth.a; # (n.h)^2
        MUL     ndoth.a, ndoth.a, ndoth.a; # (n.h)^4
        MAD     light.xyz, lightraw, {2.0, 2.0, 2.0}, {-1.0, -1.0, -1.0};
        DP3_SAT ndotl.a, norm, light;
        ADD     ndotl.a, ndotl.a, lightraw.a;
        MAD     result.color.xyz, ndotl.a, color, ndoth.a;
        MOV     result.color.a, 1.;
        END
    )";
  static type& instance() {
    static type& f = *new type;
    return f;
  }
};

// *** ENV_DOT3 extension
// http://www.ati.com/developer/sdk/RadeonSDK/Html/Samples/OpenGL/RadeonSimpleDOT3.html
// The main difference between the texture_env_* extension and the combiners is
// that the combiners replace the entire fragment pipeline, while the
// texture_env_* extensions only extend it. So the texture_env_dot3 extension
// only adds an additional env mode to the texture_env_combine extension:
// DOT3_RGB_EXT                    Arg0 <dotprod> Arg1
// (this is just like MODULATE or REPLACE)
// where arg0 and arg1 are: PRIMARY_COLOR_EXT, TEXTURE, CONSTANT_EXT or PREVIOUS_EXT.
struct NormalMapping_dot3 final : NormalMapping {
  using type = NormalMapping_dot3;
  string name() const override { return "dot3"; }
  bool is_supported() const override {
    if (!contains(gl_extensions_string(), "GL_ARB_texture_env_dot3")) return false;
    GLint max_texture_units;
    glGetIntegerv(GL_MAX_TEXTURE_UNITS_ARB, &max_texture_units);
    if (getenv_bool("OPENGL_DEBUG")) SHOW(max_texture_units);
    if (max_texture_units < 2) return false;
    return true;
  }
  void init() override {
    USE_GL_EXT(glActiveTextureARB, PFNGLACTIVETEXTUREARBPROC);
    // Note, that you are not using the second tmu, only the second combiner,
    // but you have to enable the tmu and set a valid configuration.
    // I usually bind a tiny white texture.
    {
      glActiveTextureARB(GL_TEXTURE1_ARB);
      glEnable(GL_TEXTURE_2D);
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      glMatrixMode(GL_TEXTURE);
      glLoadIdentity();
      glMatrixMode(GL_MODELVIEW);
      GLuint texname1;
      glGenTextures(1, &texname1);
      glBindTexture(GL_TEXTURE_2D, texname1);
      Image itexture(V(2, 2), Pixel::black());
      int level = 0, border = 0;
      GLenum internal_format = GL_RGBA8;
      glTexImage2D(GL_TEXTURE_2D, level, internal_format, itexture.xsize(), itexture.ysize(), border, GL_RGBA,
                   GL_UNSIGNED_BYTE, itexture.data());
      glDisable(GL_TEXTURE_GEN_S);
      glDisable(GL_TEXTURE_GEN_T);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    // TMU0:
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE_EXT);
    // dst.rgb = N dot3 L
    glTexEnvf(GL_TEXTURE_ENV, GL_COMBINE_RGB_EXT, GL_DOT3_RGBA_EXT);
    glTexEnvf(GL_TEXTURE_ENV, GL_RGB_SCALE_EXT, 1.0f);
    // arg0 = N
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB_EXT, GL_TEXTURE0_ARB);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_RGB_EXT, GL_SRC_COLOR);
    // arg1 = L
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE1_RGB_EXT, GL_CONSTANT_EXT);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND1_RGB_EXT, GL_SRC_COLOR);
    //
    // TMU1:
    glActiveTextureARB(GL_TEXTURE1_ARB);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE_EXT);
    // dst.rgb = scale * arg0 * arg1
    glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB_EXT, GL_MODULATE);
    glTexEnvf(GL_TEXTURE_ENV, GL_RGB_SCALE_EXT, 1.0f);
    // arg0 = result of the previous combiner
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB_EXT, GL_PREVIOUS_EXT);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_RGB_EXT, GL_SRC_COLOR);
    // arg1 = primary color
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE1_RGB_EXT, GL_PRIMARY_COLOR_EXT);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND1_RGB_EXT, GL_SRC_COLOR);
    //
    glActiveTextureARB(GL_TEXTURE0_ARB);
  }
  void set_parameters(const Vector& lightdirmodel, const Vector& eyedirmodel, float ambient, float lightsource,
                      const Pixel& meshcolor_s) override {
    dummy_use(eyedirmodel, ambient, meshcolor_s);
    Vector dot3_scaled_light = lightdirmodel * lightsource / .85f;
    Vec4<float> light2 = normalizef(dot3_scaled_light);
    glTexEnvfv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, light2.data());
  }
  void activate() override { glDisable(GL_BLEND); }
  void deactivate() override { glEnable(GL_BLEND); }
  Array<uint8_t> fill_buffer_rgb8(const Image& image) {
    Array<uint8_t> buffer;
    buffer.reserve(assert_narrow_cast<int>(product(image.dims()) * 3));
    for_int(y, image.ysize()) {
      for_int(x, image.xsize()) {
        if (image.zsize() == 1) {
          for_int(z, 3) buffer.push(image[y][x][0]);
        } else {
          for_int(z, 3) buffer.push(image[y][x][z]);
        }
      }
    }
    return buffer;
  }
  static type& instance() {
    static type& f = *new type;
    return f;
  }
};

struct NormalMapping_nvrc final : NormalMapping {
  using type = NormalMapping_nvrc;
  string name() const override { return "nvrc"; }
  bool is_supported() const override { return contains(gl_extensions_string(), "GL_NV_register_combiners"); }
  void init() override {
    USE_GL_EXT(glCombinerParameteriNV, PFNGLCOMBINERPARAMETERINVPROC);
    USE_GL_EXT(glCombinerInputNV, PFNGLCOMBINERINPUTNVPROC);
    USE_GL_EXT(glCombinerOutputNV, PFNGLCOMBINEROUTPUTNVPROC);
    USE_GL_EXT(glFinalCombinerInputNV, PFNGLFINALCOMBINERINPUTNVPROC);
    glCombinerParameteriNV(GL_NUM_GENERAL_COMBINERS_NV, 2);
    // Combiner0:
    //  SP0 = (n * l)
    //  SP1 = (n * h)
    // Combiner1:
    //  SP0 = (n * l)c + a * c
    // FinalCombiner:
    //  E = (n * h)
    //  F = (n * h)
    //  OUT = (n * h)^4 + (n * l + a)c
    //
    // input: stage, portion, variable,  input, mapping, componentUsage
    // A = texel normal  (2 * v - 1)
    glCombinerInputNV(GL_COMBINER0_NV, GL_RGB, GL_VARIABLE_A_NV, GL_TEXTURE0_ARB, GL_EXPAND_NORMAL_NV, GL_RGB);
    // B = C0 = light    (2 * v - 1)
    glCombinerInputNV(GL_COMBINER0_NV, GL_RGB, GL_VARIABLE_B_NV, GL_CONSTANT_COLOR0_NV, GL_EXPAND_NORMAL_NV, GL_RGB);
    // C = texel normal  (2 * v - 1)
    glCombinerInputNV(GL_COMBINER0_NV, GL_RGB, GL_VARIABLE_C_NV, GL_TEXTURE0_ARB, GL_EXPAND_NORMAL_NV, GL_RGB);
    // D = C1 = vhalf = (light + eye) / 2  (2 * v - 1)
    glCombinerInputNV(GL_COMBINER0_NV, GL_RGB, GL_VARIABLE_D_NV, GL_CONSTANT_COLOR1_NV, GL_EXPAND_NORMAL_NV, GL_RGB);
    // out: stage, portion,  abOut, cdOut, sumOut,  scale, bias,  abDot, cdDot, muxSum
    // SPARE0 = A dot B = dot(normal * light)
    // SPARE1 = C dot D = dot(normal * vhalf)
    glCombinerOutputNV(GL_COMBINER0_NV, GL_RGB, GL_SPARE0_NV, GL_SPARE1_NV, GL_DISCARD_NV, GL_NONE, GL_NONE, GL_TRUE,
                       GL_TRUE, GL_FALSE);
    //
    // A = SPARE0 = dot(normal * light)
    glCombinerInputNV(GL_COMBINER1_NV, GL_RGB, GL_VARIABLE_A_NV, GL_SPARE0_NV, GL_UNSIGNED_IDENTITY_NV, GL_RGB);
    // B = color
    glCombinerInputNV(GL_COMBINER1_NV, GL_RGB, GL_VARIABLE_B_NV, GL_PRIMARY_COLOR_NV, GL_UNSIGNED_IDENTITY_NV, GL_RGB);
    // C = ambient
    glCombinerInputNV(GL_COMBINER1_NV, GL_RGB, GL_VARIABLE_C_NV, GL_CONSTANT_COLOR0_NV, GL_UNSIGNED_IDENTITY_NV,
                      GL_ALPHA);
    // D = color
    glCombinerInputNV(GL_COMBINER1_NV, GL_RGB, GL_VARIABLE_D_NV, GL_PRIMARY_COLOR_NV, GL_UNSIGNED_IDENTITY_NV, GL_RGB);
    // SPARE0 = A * B + C * D = (dot(normal * light) + ambient) * color
    glCombinerOutputNV(GL_COMBINER1_NV, GL_RGB, GL_DISCARD_NV, GL_DISCARD_NV, GL_SPARE0_NV, GL_NONE, GL_NONE, GL_FALSE,
                       GL_FALSE, GL_FALSE);
    //
    // E = SPARE1 = dot(normal * vhalf)
    glFinalCombinerInputNV(GL_VARIABLE_E_NV, GL_SPARE1_NV, GL_UNSIGNED_IDENTITY_NV, GL_RGB);
    // F = SPARE1 = dot(normal * vhalf)
    glFinalCombinerInputNV(GL_VARIABLE_F_NV, GL_SPARE1_NV, GL_UNSIGNED_IDENTITY_NV, GL_RGB);
    // A = E*F = dot(normal * vhalf) ** 2
    glFinalCombinerInputNV(GL_VARIABLE_A_NV, GL_E_TIMES_F_NV, GL_UNSIGNED_IDENTITY_NV, GL_RGB);
    // B = E*F = dot(normal * vhalf) ** 2
    glFinalCombinerInputNV(GL_VARIABLE_B_NV, GL_E_TIMES_F_NV, GL_UNSIGNED_IDENTITY_NV, GL_RGB);
    // D = SPARE0 = (dot(normal * light) + ambient) * color
    glFinalCombinerInputNV(GL_VARIABLE_D_NV, GL_SPARE0_NV, GL_UNSIGNED_IDENTITY_NV, GL_RGB);
    // C = ZERO
    glFinalCombinerInputNV(GL_VARIABLE_C_NV, GL_ZERO, GL_UNSIGNED_IDENTITY_NV, GL_RGB);
    // OUT = A * B + (1 - A) * C + D
    //     = dot(normal * vhalf) ** 4 + dot(normal * light) * color
    //
    // replace diffuse term by ambient:
    // glFinalCombinerInputNV(GL_VARIABLE_D_NV, GL_CONSTANT_COLOR1_NV, GL_UNSIGNED_IDENTITY_NV, GL_RGB);
  }
  void set_parameters(const Vector& lightdirmodel, const Vector& eyedirmodel, float ambient, float lightsource,
                      const Pixel& meshcolor_s) override {
    Vector vhalf = ok_normalized(lightdirmodel + eyedirmodel);
    // Vector scaled_light = lightdirmodel * (lambient / .6f);
    // Vector scaled_vhalf = vhalf * .93f * (lightsource / .75f);
    Vector scaled_light = lightdirmodel * lightsource;
    // just pull the Red channel out; (default is gray 0.5f)
    float meshspecular = meshcolor_s[0] / 255.f;
    assertw(meshcolor_s[0] == meshcolor_s[1]);
    assertw(meshcolor_s[0] == meshcolor_s[2]);
    const int phong = 4;
    Vector scaled_vhalf = vhalf * pow(lightsource * meshspecular, 1.f / phong);
    USE_GL_EXT(glCombinerParameterfvNV, PFNGLCOMBINERPARAMETERFVNVPROC);
    // C0 = light_vector, scalar_ambient
    Vec4<float> light2 = normalizef(scaled_light);
    light2[3] = ambient;
    glCombinerParameterfvNV(GL_CONSTANT_COLOR0_NV, light2.data());
    // C1 = vhalf_vector, 0.f
    Vec4<float> vhalf2 = normalizef(scaled_vhalf);
    glCombinerParameterfvNV(GL_CONSTANT_COLOR1_NV, vhalf2.data());
  }
  void activate() override {
    // const int GL_REGISTER_COMBINERS_NV = 0x8522;
    glEnable(GL_REGISTER_COMBINERS_NV);
  }
  void deactivate() override {
    // const int GL_REGISTER_COMBINERS_NV = 0x8522;
    glDisable(GL_REGISTER_COMBINERS_NV);
  }
  Array<uint8_t> fill_buffer_rgb8(const Image& image) {
    Array<uint8_t> buffer;
    buffer.reserve(assert_narrow_cast<int>(product(image.dims()) * 3));
    for_int(y, image.ysize()) {
      for_int(x, image.xsize()) {
        if (image.zsize() == 1) {
          for_int(z, 3) buffer.push(image[y][x][0]);
        } else {
          for_int(z, 3) buffer.push(image[y][x][z]);
        }
      }
    }
    return buffer;
  }
  static type& instance() {
    static type& f = *new type;
    return f;
  }
};

NormalMapping* NormalMapping::get() {
  static Array<NormalMapping*> normalmappings;
  static std::once_flag flag;
  std::call_once(flag, [] {
    normalmappings.push(&NormalMapping_ogl2::instance());
    normalmappings.push(&NormalMapping_frag1::instance());
    normalmappings.push(&NormalMapping_nvrc::instance());
    normalmappings.push(&NormalMapping_dot3::instance());
  });
  assertx(normalmappings.num());
  string desired_name = getenv_string("NORMAL_MAPPING");
  for (NormalMapping* normalmapping : normalmappings) {
    string fname = normalmapping->name();
    if (desired_name != "") {
      if (fname == desired_name) {
        assertx(normalmapping->is_supported());
        return normalmapping;
      }
    } else {
      if (normalmapping->is_supported()) return normalmapping;
    }
  }
  if (desired_name != "") assertnever("NormalMapping '" + desired_name + "' not recognized");
  return nullptr;  // normal mapping not supported at all
}

}  // namespace hh
