// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "HW.h"
#include "libHh/GMesh.h"
#include "libHh/SRMesh.h"
#include "libHh/Stat.h"

namespace hh {

void SRMesh::ogl_process_materials() {
  _ogl_mat_byte_rgba.reserve(_materials.num());
  for_int(matid, _materials.num()) {
    const string& str = _materials.get(matid);
    Vec3<float> co;
    if (!parse_key_vec(str.c_str(), "rgb", co)) {
      // co = V(.8f, .5f, .4f);
      co = V(.6f, .6f, .6f);
    }
    _ogl_mat_byte_rgba.push(
        Pixel(uint8_t(co[0] * 255.f + .5f), uint8_t(co[1] * 255.f + .5f), uint8_t(co[2] * 255.f + .5f)));
  }
}

void SRMesh::ogl_render_faces_individually(bool unlit_texture) {
  if (!_ogl_mat_byte_rgba.num()) ogl_process_materials();
  glBegin(GL_TRIANGLES);
  if (unlit_texture) {
    for (SRAFace* f : HH_ELIST_RANGE(_active_faces, SRAFace, activef)) {
#if defined(SR_SW_CULLING)
      if (!f->vertices[0]->visible && !f->vertices[1]->visible && !f->vertices[2]->visible) continue;
#endif
      const SRAVertex* v0 = f->vertices[0];
      glVertex3fv(get_point(v0).data());
      const SRAVertex* v1 = f->vertices[1];
      glVertex3fv(get_point(v1).data());
      const SRAVertex* v2 = f->vertices[2];
      glVertex3fv(get_point(v2).data());
    }
  } else {
    int omatid = -1;
    for (SRAFace* f : HH_ELIST_RANGE(_active_faces, SRAFace, activef)) {
#if defined(SR_SW_CULLING)
      if (!f->vertices[0]->visible && !f->vertices[1]->visible && !f->vertices[2]->visible) continue;
#endif
      int matid = f->matid;  // material of f
      if (matid != omatid) {
        omatid = matid;
        int rmatid = matid & ~k_Face_visited_mask;  // strip off high bit
        glColor4ubv(_ogl_mat_byte_rgba[rmatid].data());
      }
      const SRAVertex* v0 = f->vertices[0];
      glNormal3fv(get_normal(v0).data());
      glVertex3fv(get_point(v0).data());
      const SRAVertex* v1 = f->vertices[1];
      glNormal3fv(get_normal(v1).data());
      glVertex3fv(get_point(v1).data());
      const SRAVertex* v2 = f->vertices[2];
      glNormal3fv(get_normal(v2).data());
      glVertex3fv(get_point(v2).data());
    }
  }
  glEnd();
}

inline void SRMesh::draw_vertex(const SRAVertex* v, bool use_texture) const {
  if (!use_texture) glNormal3fv(get_normal(v).data());
  glVertex3fv(get_point(v).data());
}

template <bool use_texture> void SRMesh::ogl_render_faces_strips_aux() {
  _cur_frame_mask = _cur_frame_mask ^ k_Face_visited_mask;
  const unsigned lcur_frame_mask = _cur_frame_mask;
  int omatid = -1;
  dummy_use(omatid);
  const EListNode* ndelim = _active_faces.delim();
  assertx(offsetof(SRAFace, activef) == 0);
  for (EListNode* n = ndelim->next();;) {
    if (n == ndelim) break;
    SRAFace* f = HH_ELIST_OUTER(SRAFace, activef, n);
    n = n->next();
    int matid = f->matid;
    if ((unsigned(matid) & k_Face_visited_mask) == lcur_frame_mask) continue;
    int matidv = matid ^ k_Face_visited_mask;  // "material id visited"
    f->matid = matidv;
    if (!use_texture && matid != omatid) {
      omatid = matid;
      int rmatid = (matid & matidv);  // strip off high bit
      glColor4ubv(_ogl_mat_byte_rgba[rmatid].data());
    }
    const SRAVertex* v1n;
    const SRAVertex* v2n;
    SRAFace* fn;  // next neighboring face
#if defined(SR_SW_CULLING)
    int sw_culling_3bits =
        ((f->vertices[0]->visible << 0) | (f->vertices[1]->visible << 1) | (f->vertices[2]->visible << 2));
    if (!sw_culling_3bits) continue;
#endif
    // Here I explored (971220) only trying the first neighboring face
    //  test, hoping that it would reduce computation.
    // However, the reduction (only 0.7% of total using HPIXPC) was
    //  more than offset by the increase in vertex transforms in OpenGL
    //  (1.3% of total cycles)    (tstrips: 4.21 -> 2.8 faces/strip)
    if (fn = f->fnei[0], fn->matid == matid) {
      glBegin(GL_TRIANGLE_STRIP);
      draw_vertex(f->vertices[0], use_texture);
      draw_vertex(f->vertices[1], use_texture);
      draw_vertex(f->vertices[2], use_texture);
      v1n = f->vertices[2];
      v2n = f->vertices[1];
#if defined(SR_SW_CULLING)
      // 210 -> 210
      sw_culling_3bits = sw_culling_3bits;
#endif
    } else if (fn = f->fnei[1], fn->matid == matid) {
      glBegin(GL_TRIANGLE_STRIP);
      draw_vertex(f->vertices[1], use_texture);
      draw_vertex(f->vertices[2], use_texture);
      draw_vertex(f->vertices[0], use_texture);
      v1n = f->vertices[0];
      v2n = f->vertices[2];
#if defined(SR_SW_CULLING)
      // 210 -> 02d
      sw_culling_3bits = (sw_culling_3bits >> 1) | ((sw_culling_3bits & 0x1) << 2);
#endif
    } else if (fn = f->fnei[2], fn->matid == matid) {
      glBegin(GL_TRIANGLE_STRIP);
      draw_vertex(f->vertices[2], use_texture);
      draw_vertex(f->vertices[0], use_texture);
      draw_vertex(f->vertices[1], use_texture);
      v1n = f->vertices[1];
      v2n = f->vertices[0];
#if defined(SR_SW_CULLING)
      // 210 -> 10d
      sw_culling_3bits = ((sw_culling_3bits & 0x3) << 1);
#endif
    } else {
      glBegin(GL_TRIANGLES);
      draw_vertex(f->vertices[0], use_texture);
      draw_vertex(f->vertices[1], use_texture);
      draw_vertex(f->vertices[2], use_texture);
      glEnd();
      continue;
    }
    ASSERTX(fn);
    // Unrolled this loop twice in order to get rid of "v1oldest" that
    //  was used to control alternating left/right strip formation.
    for (;;) {
      // *** First iteration of loop.
      {
        fn->matid = matidv;
        SRAVertex** pvrand = fn->vertices.data();
        SRAVertex* vrand = *pvrand;
        pvrand += (vrand == v2n) + (vrand == v1n) * 2;
        SRAFace** pfnei = &fn->fnei[2];
        pfnei -= (vrand == v1n) + (vrand == v2n) * 2;
        v2n = *pvrand;
#if defined(SR_SW_CULLING)
        sw_culling_3bits = (sw_culling_3bits >> 1) | (v2n->visible << 2);
        if (!sw_culling_3bits) break;
#endif
        fn = *pfnei;
        draw_vertex(v2n, use_texture);
        if (fn->matid != matid) break;
      }
      // *** Second iteration of loop.
      {
        fn->matid = matidv;
        SRAVertex** pvrand = fn->vertices.data();
        SRAVertex* vrand = *pvrand;
        pvrand += (vrand == v2n) + (vrand == v1n) * 2;
        SRAFace** pfnei = &fn->fnei[1];
        pfnei += (vrand == v2n) - (vrand == v1n);
        v1n = *pvrand;
#if defined(SR_SW_CULLING)
        sw_culling_3bits = (sw_culling_3bits >> 1) | (v2n->visible << 2);
        if (!sw_culling_3bits) break;
#endif
        fn = *pfnei;
        draw_vertex(v1n, use_texture);
        if (fn->matid != matid) break;
      }
    }
    glEnd();
  }
}

void SRMesh::ogl_render_faces_strips(bool unlit_texture) {
  if (!_ogl_mat_byte_rgba.num()) ogl_process_materials();
  ASSERTX(verify_all_faces_visited());
  if (unlit_texture) {
    ogl_render_faces_strips_aux<true>();
  } else {
    ogl_render_faces_strips_aux<false>();
  }
  ASSERTX(verify_all_faces_visited());
}

int SRMesh::ogl_render_striplines() {
  if (!_ogl_mat_byte_rgba.num()) ogl_process_materials();
  ASSERTX(verify_all_faces_visited());
  int ntstrips = 0;
  // doesn't need to be all that fast, so not carefully optimized.
  _cur_frame_mask = _cur_frame_mask ^ k_Face_visited_mask;
  for (SRAFace* ff : HH_ELIST_RANGE(_active_faces, SRAFace, activef)) {
    SRAFace* f = ff;       // current face
    int matid = f->matid;  // material of f
    SRAFace* fn;           // next neighboring face
    if ((matid & k_Face_visited_mask) == _cur_frame_mask) continue;
    ntstrips++;
    int matidv = matid ^ k_Face_visited_mask;
    f->matid = matidv;
    const SRAVertex* v1n;
    const SRAVertex* v2n;
    const SRAVertex* vn;
    if (0) {
    } else if (fn = f->fnei[0], HH_ASSUME(fn), fn->matid == matid) {
      fn->matid = matidv;
      v1n = f->vertices[0];
      v2n = f->vertices[1];
      vn = f->vertices[2];
    } else if (fn = f->fnei[1], HH_ASSUME(fn), fn->matid == matid) {
      fn->matid = matidv;
      v1n = f->vertices[1];
      v2n = f->vertices[2];
      vn = f->vertices[0];
    } else if (fn = f->fnei[2], HH_ASSUME(fn), fn->matid == matid) {
      fn->matid = matidv;
      v1n = f->vertices[2];
      v2n = f->vertices[0];
      vn = f->vertices[1];
    } else {
      fn = nullptr;
      v1n = f->vertices[0];
      v2n = f->vertices[1];
      vn = f->vertices[2];
    }
    glBegin(GL_LINE_STRIP);
    Point p = interp(get_point(v1n), get_point(v2n), get_point(vn));
    glVertex3fv(p.data());
    v1n = vn;
    bool v1oldest = false;
    int striplen = 1;
    for (;;) {
      if (!fn) break;
      striplen++;
      p = interp(get_point(fn->vertices[0]), get_point(fn->vertices[1]), get_point(fn->vertices[2]));
      glVertex3fv(p.data());
      const SRAVertex* vrand = fn->vertices[0];
      if (vrand == v2n) {
        vn = fn->vertices[1];
        if (v1oldest) {
          fn = fn->fnei[2];
          if (fn->matid != matid) {
            fn = nullptr;
          } else {
            fn->matid = matidv;
            v1n = vn;
            v1oldest = false;
          }
        } else {
          fn = fn->fnei[0];
          if (fn->matid != matid) {
            fn = nullptr;
          } else {
            fn->matid = matidv;
            v2n = vn;
            v1oldest = true;
          }
        }
      } else if (vrand == v1n) {
        vn = fn->vertices[2];
        if (v1oldest) {
          fn = fn->fnei[0];
          if (fn->matid != matid) {
            fn = nullptr;
          } else {
            fn->matid = matidv;
            v1n = vn;
            v1oldest = false;
          }
        } else {
          fn = fn->fnei[1];
          if (fn->matid != matid) {
            fn = nullptr;
          } else {
            fn->matid = matidv;
            v2n = vn;
            v1oldest = true;
          }
        }
      } else {
        vn = vrand;
        if (v1oldest) {
          fn = fn->fnei[1];
          if (fn->matid != matid) {
            fn = nullptr;
          } else {
            fn->matid = matidv;
            v1n = vn;
            v1oldest = false;
          }
        } else {
          fn = fn->fnei[2];
          if (fn->matid != matid) {
            fn = nullptr;
          } else {
            fn->matid = matidv;
            v2n = vn;
            v1oldest = true;
          }
        }
      }
    }
    glEnd();
    HH_SSTAT(SSRstriplen, striplen);
  }
  ASSERTX(verify_all_faces_visited());
  return ntstrips;
}

static inline void sr_draw_segment(const SRAVertex* v0, const SRAVertex* v1, bool hasfn, const Point& p0,
                                   const Point& p1) {
  if (v0 > v1 && hasfn) return;
  glVertex3fv(p0.data());
  glVertex3fv(p1.data());
}

void SRMesh::ogl_render_edges() {
  glBegin(GL_LINES);
  for (SRAFace* f : HH_ELIST_RANGE(_active_faces, SRAFace, activef)) {
    const SRAVertex* v0 = f->vertices[0];
    const SRAVertex* v1 = f->vertices[1];
    const SRAFace* fn2 = f->fnei[2];
    const Point& p0 = get_point(v0);
    const Point& p1 = get_point(v1);
    sr_draw_segment(v0, v1, fn2 != &_isolated_aface, p0, p1);
    const SRAVertex* v2 = f->vertices[2];
    const SRAFace* fn0 = f->fnei[0];
    const Point& p2 = get_point(v2);
    sr_draw_segment(v1, v2, fn0 != &_isolated_aface, p1, p2);
    const SRAFace* fn1 = f->fnei[1];
    sr_draw_segment(v2, v0, fn1 != &_isolated_aface, p2, p0);
  }
  glEnd();
}

void SRMesh::ogl_show_radii() {
  for (SRAVertex* v : HH_ELIST_RANGE(_active_vertices, SRAVertex, activev)) {
    if (!splitable(v)) continue;
    const Point& p = get_point(v);
    glRasterPos3fv(p.data());
    float radius = -get_radiusneg(v);
    string s = sform("%g", radius);
    glCallLists(narrow_cast<int>(s.size()), GL_UNSIGNED_BYTE, reinterpret_cast<const uchar*>(s.c_str()));
  }
}

void SRMesh::ogl_show_residuals(bool uniform_too) {
  glBegin(GL_LINES);
  for (SRAVertex* v : HH_ELIST_RANGE(_active_vertices, SRAVertex, activev)) {
    if (!splitable(v)) continue;
    if (1) {
      float dir_error_mag = sqrt(get_dir_error_mag2(v));
      Vector dir_error_dir = dir_error_mag * get_normal(v);
      Point p1 = get_point(v) - dir_error_dir;
      Point p2 = get_point(v) + dir_error_dir;
      glVertex3fv(p1.data());
      glVertex3fv(p2.data());
    }
    if (uniform_too) {
      const Vector& dir = get_normal(v);
      const float uni_error_mag = sqrt(get_uni_error_mag2(v));
      Vector v1;
      if (dir[2] < .5f) {
        const float d = 1.f / sqrt(square(dir[0]) + square(dir[1]));
        v1 = Vector(-dir[1] * d, dir[0] * d, 0.f);
      } else {
        const float d = 1.f / sqrt(square(dir[1]) + square(dir[2]));
        v1 = Vector(0.f, -dir[2] * d, dir[1] * d);
      }
      const Vector v2 = cross(v1, dir) * uni_error_mag;
      v1 *= uni_error_mag;
      {
        Point p1 = get_point(v) - v1, p2 = get_point(v) + v1;
        glVertex3fv(p1.data());
        glVertex3fv(p2.data());
      }
      {
        Point p1 = get_point(v) - v2, p2 = get_point(v) + v2;
        glVertex3fv(p1.data());
        glVertex3fv(p2.data());
      }
    }
  }
  glEnd();
}

// *** TVC

constexpr int k_cache_size = 16;

int SRMesh::ogl_render_tvclines() {
  if (!_ogl_mat_byte_rgba.num()) ogl_process_materials();
  ASSERTX(verify_all_faces_visited());
  ASSERTX(verify_all_vertices_uncached());
  int cache_time = _cache_time;
  const int desiredloc = k_cache_size - 9;
  _cur_frame_mask = _cur_frame_mask ^ k_Face_visited_mask;
  const unsigned lcur_frame_mask = _cur_frame_mask;
  int omatid = -1;
  dummy_use(omatid);
  const EListNode* ndelim = _active_faces.delim();
  assertx(offsetof(SRAFace, activef) == 0);
  Point p;
  for (EListNode* n = ndelim->next();;) {
  GOTO_STRIP_RESTART_FROM_SCRATCH:
    if (n == ndelim) break;
    SRAFace* f = HH_ELIST_OUTER(SRAFace, activef, n);
    n = n->next();
    int matid = f->matid;
    if ((unsigned(matid) & k_Face_visited_mask) == lcur_frame_mask) continue;
    int matidv = matid ^ k_Face_visited_mask;  // material id visited
    // index j of Corner c within current face
    int j = 0;
    SRAFace* fnext;
    int jnext;
    int cache_time_next;
    glBegin(GL_LINE_STRIP);
    for (;;) {    // form one strip
      for (;;) {  // form ring(s) while having no fnext
        f->matid = matidv;
        SRAVertex* v0 = f->vertices[j];
        SRAVertex* v1 = f->vertices[mod3(j + 1)];
        SRAVertex* v2 = f->vertices[mod3(j + 2)];
        if (v0->cached_time < cache_time) v0->cached_time = (cache_time++) + k_cache_size;
        if (v1->cached_time < cache_time) v1->cached_time = (cache_time++) + k_cache_size;
        if (v2->cached_time < cache_time) v2->cached_time = (cache_time++) + k_cache_size;
        p = interp(get_point(v0), get_point(v1), get_point(v2));
        glVertex3fv(p.data());
        SRAFace* fint = f->fnei[mod3(j + 1)];
        SRAFace* fext = f->fnei[j];
        if (fint->matid == matid) {
          f = fint;
          j = get_vf_j0(v0, f);
          if (fext->matid == matid) {
            fnext = fext;
            jnext = get_vf_j0(v2, fext);
            cache_time_next = cache_time;
            break;
          }
        } else if (fext->matid == matid) {
          f = fext;
          j = get_vf_j0(v2, f);
        } else {
          glEnd();
          goto GOTO_STRIP_RESTART_FROM_SCRATCH;  // "continue" on outermost loop
        }
      }
      for (;;) {  // form ring(s) while having fnext
        f->matid = matidv;
        SRAVertex* v0 = f->vertices[j];
        SRAVertex* v1 = f->vertices[mod3(j + 1)];
        SRAVertex* v2 = f->vertices[mod3(j + 2)];
        if (v0->cached_time < cache_time) v0->cached_time = (cache_time++) + k_cache_size;
        if (v1->cached_time < cache_time) v1->cached_time = (cache_time++) + k_cache_size;
        if (v2->cached_time < cache_time) v2->cached_time = (cache_time++) + k_cache_size;
        p = interp(get_point(v0), get_point(v1), get_point(v2));
        glVertex3fv(p.data());
        SRAFace* fint = f->fnei[mod3(j + 1)];
        if (fint->matid == matid) {
          f = fint;
          j = get_vf_j0(v0, f);
          continue;
        }
        SRAFace* fext = f->fnei[j];
        if (fext->matid != matid) break;
        f = fext;
        j = get_vf_j0(v2, f);
        // Decide to add next ring (e.g. 2 faces) or restart.
        int nf = 1;
        {
          SRAFace* ff = f;
          int jj = j;
          for (;;) {
            ff = ff->fnei[mod3(jj + 1)];
            if (ff->matid != matid) break;
            nf++;
            jj = get_vf_j0(v2, ff);
          }
        }
        if (cache_time - cache_time_next + (nf - 1) > desiredloc) break;
      }
      glEnd();
      if (fnext->matid != matid) break;
      f = fnext;
      j = jnext;
      glBegin(GL_LINE_STRIP);
    }
  }
  HH_SSTAT(Stvc_vpt0, float(cache_time - _cache_time) / num_active_faces());
  int nmiss = cache_time - _cache_time;
  if (0) SHOW(nmiss);
  _cache_time = cache_time + k_cache_size;
  ASSERTX(verify_all_faces_visited());
  ASSERTX(verify_all_vertices_uncached());
  return nmiss;
}

void SRMesh::ogl_render_faces_tvc(bool unlit_texture) {
  dummy_use(unlit_texture);
  if (!_ogl_mat_byte_rgba.num()) ogl_process_materials();
  ASSERTX(verify_all_faces_visited());
  ASSERTX(verify_all_vertices_uncached());
  int cache_time = _cache_time;
  const int desiredloc = k_cache_size - 9;
  _cur_frame_mask = _cur_frame_mask ^ k_Face_visited_mask;
  const unsigned lcur_frame_mask = _cur_frame_mask;
  int omatid = -1;
  dummy_use(omatid);
  const EListNode* ndelim = _active_faces.delim();
  assertx(offsetof(SRAFace, activef) == 0);
  glBegin(GL_TRIANGLES);
  for (EListNode* n = ndelim->next();;) {
  GOTO_STRIP_RESTART_FROM_SCRATCH:
    if (n == ndelim) break;
    SRAFace* f = HH_ELIST_OUTER(SRAFace, activef, n);
    n = n->next();
    int matid = f->matid;
    if ((unsigned(matid) & k_Face_visited_mask) == lcur_frame_mask) continue;
    int matidv = matid ^ k_Face_visited_mask;  // material id visited
#if !defined(SR_USE_TEXTURE)
    if (matid != omatid) {
      omatid = matid;
      int rmatid = (matid & matidv);  // strip off high bit
      glColor4ubv(_ogl_mat_byte_rgba[rmatid].data());
    }
#endif
    int j = 0;  // index j of corner within current face f
    SRAFace* fnext;
    int jnext;
    int cache_time_next;
    // NOTE: Measuring vertex caching on indexed strips gives slightly
    //  fewer cache misses than measuring on indexed triangles.
    SRAVertex* v0 = f->vertices[0];  // next to Corner c
    SRAVertex* v1 = f->vertices[1];
    if (v0->cached_time < cache_time) v0->cached_time = (cache_time++) + k_cache_size;
    if (v1->cached_time < cache_time) v1->cached_time = (cache_time++) + k_cache_size;
    SRAVertex* v2 = f->vertices[2];
    for (;;) {    // form one strip
      for (;;) {  // form ring(s) while having no fnext
        f->matid = matidv;
        ASSERTX(v0 == f->vertices[mod3(j + 0)]);
        ASSERTX(v1 == f->vertices[mod3(j + 1)]);
        ASSERTX(v2 == f->vertices[mod3(j + 2)]);
        glNormal3fv(get_normal(v0).data());
        glVertex3fv(get_point(v0).data());
        glNormal3fv(get_normal(v1).data());
        glVertex3fv(get_point(v1).data());
        glNormal3fv(get_normal(v2).data());
        glVertex3fv(get_point(v2).data());
        if (v2->cached_time < cache_time) v2->cached_time = (cache_time++) + k_cache_size;
        SRAFace* fint = f->fnei[mod3(j + 1)];
        SRAFace* fext = f->fnei[j];
        if (fint->matid == matid) {
          f = fint;
          j = get_vf_j0(v0, f);
          v1 = v2;
          v2 = f->vertices[mod3(j + 2)];
          if (fext->matid == matid) {
            fnext = fext;
            jnext = get_vf_j0(v1, fext);
            cache_time_next = cache_time;
            break;
          }
        } else if (fext->matid == matid) {
          f = fext;
          j = get_vf_j0(v2, f);
          v0 = v2;
          v2 = f->vertices[mod3(j + 2)];
        } else {
          goto GOTO_STRIP_RESTART_FROM_SCRATCH;  // "continue" on outermost loop
        }
      }
      for (;;) {  // form ring(s) while having fnext
        f->matid = matidv;
        ASSERTX(v0 == f->vertices[mod3(j + 0)]);
        ASSERTX(v1 == f->vertices[mod3(j + 1)]);
        ASSERTX(v2 == f->vertices[mod3(j + 2)]);
        glNormal3fv(get_normal(v0).data());
        glVertex3fv(get_point(v0).data());
        glNormal3fv(get_normal(v1).data());
        glVertex3fv(get_point(v1).data());
        glNormal3fv(get_normal(v2).data());
        glVertex3fv(get_point(v2).data());
        if (v2->cached_time < cache_time) v2->cached_time = (cache_time++) + k_cache_size;
        SRAFace* fint = f->fnei[mod3(j + 1)];
        if (fint->matid == matid) {
          f = fint;
          j = get_vf_j0(v0, f);
          v1 = v2;
          v2 = f->vertices[mod3(j + 2)];
          continue;
        }
        SRAFace* fext = f->fnei[j];
        if (fext->matid != matid) break;
        f = fext;
        j = get_vf_j0(v2, f);
        v0 = v2;
        v2 = f->vertices[mod3(j + 2)];
        // Decide to add next ring (e.g. 2 faces) or restart.
        int nf = 1;
        {
          SRAFace* ff = f;
          int jj1 = mod3(j + 1);
          for (;;) {
            ff = ff->fnei[jj1];
            if (ff->matid != matid) break;
            nf++;
            jj1 = get_vf_j1(v0, ff);
          }
        }
        if (cache_time - cache_time_next + (nf - 1) > desiredloc) break;
      }
      if (fnext->matid != matid) break;
      f = fnext;
      j = jnext;
      v0 = f->vertices[j];
      v1 = f->vertices[mod3(j + 1)];
      v2 = f->vertices[mod3(j + 2)];
      if (v0->cached_time < cache_time) v0->cached_time = (cache_time++) + k_cache_size;
      if (v1->cached_time < cache_time) v1->cached_time = (cache_time++) + k_cache_size;
    }
  }
  glEnd();
  HH_SSTAT(Stvc_vpt, float(cache_time - _cache_time) / num_active_faces());
  if (0) SHOW(cache_time - _cache_time);
  _cache_time = cache_time + k_cache_size;
  ASSERTX(verify_all_faces_visited());
  ASSERTX(verify_all_vertices_uncached());
}

}  // namespace hh
