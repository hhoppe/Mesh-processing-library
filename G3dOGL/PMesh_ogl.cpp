// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "HW.h"
#include "PMesh.h"
#include "GMesh.h"              // GMesh::string_key()

namespace hh {

extern bool g_is_ati;             // defined in G3dOGL

void AWMesh::ogl_process_materials() {
    _ogl_mat_byte_rgba.reserve(_materials.num());
    for_int(matid, _materials.num()) {
        const string& str = _materials.get(matid);
        Vec3<float> co;
        if (!parse_key_vec(str.c_str(), "rgb", co)) {
            // co = V(.8f, .5f, .4f);
            co = V(.6f, .6f, .6f);
        }
        _ogl_mat_byte_rgba.push(Pixel(uint8_t(co[0]*255.f+.5f),
                                      uint8_t(co[1]*255.f+.5f),
                                      uint8_t(co[2]*255.f+.5f)));
    }
}

void AWMesh::ogl_render_faces_individually(const PMeshInfo& pminfo, int usetexture) {
    if (!_ogl_mat_byte_rgba.num()) ogl_process_materials();
    int omatid = -1;
    // Prefer texture over color if texture is enabled.
    if (pminfo._has_uv && usetexture) {
        glDisable(GL_TEXTURE_GEN_S);
        glDisable(GL_TEXTURE_GEN_T);
    }
    glBegin(GL_TRIANGLES);
    for_int(f, _faces.num()) {
        if ((f&0x7F)==0 && f && !g_is_ati) { glEnd(); glBegin(GL_TRIANGLES); omatid = -1; }
        if (!(pminfo._has_uv && usetexture) && !pminfo._has_rgb) {
            int matid = _faces[f].attrib.matid;
            if (matid!=omatid) {
                omatid = matid;
                glColor4ubv(_ogl_mat_byte_rgba[matid].data());
            }
        }
        for_int(i, 3) {
            int w = _faces[f].wedges[i];
            if (pminfo._has_uv && usetexture) {
                glTexCoord2fv(_wedges[w].attrib.uv.data());
            } else if (pminfo._has_rgb) {
                glColor3fv(_wedges[w].attrib.rgb.data());
            }
            glNormal3fv(_wedges[w].attrib.normal.data());
            int v = _wedges[w].vertex;
            glVertex3fv(_vertices[v].attrib.point.data());
        }
    }
    glEnd();
}

void AWMesh::ogl_render_faces_strips(const PMeshInfo& pminfo, int usetexture) {
    if (!_ogl_mat_byte_rgba.num()) ogl_process_materials();
    if (0) {
        for_int(f, _faces.num()) {
            assertx((_faces[f].attrib.matid&k_Face_visited_mask)==_cur_frame_mask);
            assertx(_materials.ok(_faces[f].attrib.matid&~k_Face_visited_mask));
        }
    }
    _cur_frame_mask = _cur_frame_mask^k_Face_visited_mask;
    const int lcur_frame_mask = _cur_frame_mask;
    int omatid = -1;
    int ntstrips = 0;
    const int write_ntstrips = 0; // get about 3 faces/strip on dragon3
    bool has_uv = false;
    if (pminfo._has_uv && usetexture) {
        has_uv = true;
        glDisable(GL_TEXTURE_GEN_S);
        glDisable(GL_TEXTURE_GEN_T);
    }
    if (pminfo._has_rgb) Warning("ogl_render: RGB ignored");
    for_int(f, _faces.num()) {
        int matid = _faces[f].attrib.matid;
        if ((matid&k_Face_visited_mask)==lcur_frame_mask) continue;
        int matidv = matid^k_Face_visited_mask; // "material id visited"
        _faces[f].attrib.matid = matidv;
        if (matid!=omatid) {
            omatid = matid;
            int rmatid = (matid&matidv); // strip off high bit
            if (!has_uv) glColor4ubv(_ogl_mat_byte_rgba[rmatid].data());
        }
        if (write_ntstrips) ntstrips++;
        glBegin(GL_TRIANGLE_STRIP);
        int w1n, w2n;
        {
            w1n = _faces[f].wedges[0];
            int v1n = _wedges[w1n].vertex;
            if (has_uv) glTexCoord2fv(_wedges[w1n].attrib.uv.data());
            glNormal3fv(_wedges[w1n].attrib.normal.data());
            glVertex3fv(_vertices[v1n].attrib.point.data());
            w2n = _faces[f].wedges[1];
            int v2n = _wedges[w2n].vertex;
            if (has_uv) glTexCoord2fv(_wedges[w2n].attrib.uv.data());
            glNormal3fv(_wedges[w2n].attrib.normal.data());
            glVertex3fv(_vertices[v2n].attrib.point.data());
            w1n = _faces[f].wedges[2];
            if (has_uv) glTexCoord2fv(_wedges[w1n].attrib.uv.data());
            glNormal3fv(_wedges[w1n].attrib.normal.data());
            v1n = _wedges[w1n].vertex;
            glVertex3fv(_vertices[v1n].attrib.point.data());
        }
        int fn = _fnei[f].faces[0];
        for (;;) {
            // *** First iteration of loop.
            {
                if (fn<0 || _faces[fn].attrib.matid!=matid) break;
                int wrand = _faces[fn].wedges[0];
                if (wrand==w2n) {
                    if (_faces[fn].wedges[2]!=w1n) break;
                    _faces[fn].attrib.matid = matidv;
                    w2n = _faces[fn].wedges[1];
                    fn = _fnei[fn].faces[0];
                } else if (wrand==w1n) {
                    if (_faces[fn].wedges[1]!=w2n) break;
                    _faces[fn].attrib.matid = matidv;
                    w2n = _faces[fn].wedges[2];
                    fn = _fnei[fn].faces[1];
                } else {
                    if (_faces[fn].wedges[1]!=w1n) break;
                    if (_faces[fn].wedges[2]!=w2n) break;
                    _faces[fn].attrib.matid = matidv;
                    w2n = wrand;
                    fn = _fnei[fn].faces[2];
                }
                if (has_uv) glTexCoord2fv(_wedges[w2n].attrib.uv.data());
                glNormal3fv(_wedges[w2n].attrib.normal.data());
                glVertex3fv(_vertices[_wedges[w2n].vertex].attrib.point.data());
            }
            // *** Second iteration of loop.
            {
                if (fn<0 || _faces[fn].attrib.matid!=matid) break;
                int wrand = _faces[fn].wedges[0];
                if (wrand==w2n) {
                    if (_faces[fn].wedges[2]!=w1n) break;
                    _faces[fn].attrib.matid = matidv;
                    w1n = _faces[fn].wedges[1];
                    fn = _fnei[fn].faces[2];
                } else if (wrand==w1n) {
                    if (_faces[fn].wedges[1]!=w2n) break;
                    _faces[fn].attrib.matid = matidv;
                    w1n = _faces[fn].wedges[2];
                    fn = _fnei[fn].faces[0];
                } else {
                    if (_faces[fn].wedges[1]!=w1n) break;
                    if (_faces[fn].wedges[2]!=w2n) break;
                    _faces[fn].attrib.matid = matidv;
                    w1n = wrand;
                    fn = _fnei[fn].faces[1];
                }
                if (has_uv) glTexCoord2fv(_wedges[w1n].attrib.uv.data());
                glNormal3fv(_wedges[w1n].attrib.normal.data());
                glVertex3fv(_vertices[_wedges[w1n].vertex].attrib.point.data());
            }
        }
        glEnd();
    }
    if (write_ntstrips) SHOW(ntstrips);
    if (0) {
        for_int(f, _faces.num()) {
            assertx((_faces[f].attrib.matid&k_Face_visited_mask)==_cur_frame_mask);
        }
    }
}

static inline void pm_draw_segment(int v0, int v1, int fn, const Point& p0, const Point& p1) {
    if (v0>v1 && fn>=0) return;
    glVertex3fv(p0.data());
    glVertex3fv(p1.data());
}

void AWMesh::ogl_render_edges() {
    glBegin(GL_LINES);
    for_int(f, _faces.num()) {
        if ((f&0x7F)==0 && f && !g_is_ati) { glEnd(); glBegin(GL_LINES); }
        int w0 = _faces[f].wedges[0];
        int w1 = _faces[f].wedges[1];
        int v0 = _wedges[w0].vertex;
        int v1 = _wedges[w1].vertex;
        const Point& p0 = _vertices[v0].attrib.point;
        const Point& p1 = _vertices[v1].attrib.point;
        pm_draw_segment(v0, v1, _fnei[f].faces[2], p0, p1);
        int w2 = _faces[f].wedges[2];
        int v2 = _wedges[w2].vertex;
        const Point& p2 = _vertices[v2].attrib.point;
        pm_draw_segment(v1, v2, _fnei[f].faces[0], p1, p2);
        pm_draw_segment(v2, v0, _fnei[f].faces[1], p2, p0);
    }
    glEnd();
}

} // namespace hh
