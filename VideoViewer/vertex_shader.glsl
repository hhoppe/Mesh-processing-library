// Begin C++11 raw string literal
R""(

#if !defined(MAKEFILE_DEP)

#if defined(__cplusplus)
#error We are not inside C++ string
#endif

in vec4 vertex;                 // (p.xy, uv.st), called 'attribute' in #version 130
out vec2 frag_uv;               // called 'varying' in #version 130

void main() {
    // gl_TexCoord[0] = gl_MultiTexCoord0;
    gl_Position = vec4(vertex.xy, 0.f, 1.f);
    frag_uv = vertex.zw;
    if (false) {
        float s = 1.f;
        gl_Position = vec4((gl_VertexID==0 ? vec2(-s, -s) : gl_VertexID==1 ? vec2(+s, -s) :
                            gl_VertexID==2 ? vec2(+s, +s) : gl_VertexID==3 ? vec2(-s, +s) :
                            vertex.xy), 0, 1);
        frag_uv = (gl_Position.xy+1.f)/2.f;
        frag_uv.y = 1.f-frag_uv.y; // flip Y
    }
}
#endif // !defined(MAKEFILE_DEP)

)""                             // end C++11 raw string literal
