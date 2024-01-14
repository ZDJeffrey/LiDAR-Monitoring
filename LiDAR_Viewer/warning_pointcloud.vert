#version 330 core
layout(location = 0) in vec2 aPos;
// MVP矩阵（M为默认单位矩阵）
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * vec4(aPos,0.0f,1.0f);
    gl_PointSize = 30.0f / gl_Position.z;
}
