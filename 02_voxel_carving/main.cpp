#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../3rd_party_libs/stb/stb_image.h"
#include "../3rd_party_libs/stb/stb_image_write.h"

#define IDX3D(x,y,z, res) ((z*res*res)+(y*res)+x)

struct Pixel {
    u8 r;
    u8 g;
    u8 b;
};

Voxel* generate_voxel_grid(u32 resolution, f32 side_length) {
    Voxel* voxels = (Voxel*)malloc(resolution*resolution*resolution*sizeof(voxels[0]));

    u32 index = 0;
    for (u32 z = 0; z < resolution; ++z) {
        if (z % 32 == 0) {
            printf("\rGenerating: %.1f%%", z * 1.0f / (resolution-1) * 100);
            fflush(stdout);
        }
        for (u32 y = 0; y < resolution; ++y) {
            for (u32 x = 0; x < resolution; ++x) {
                voxels[index].value = 1.0f;
                voxels[index].position = {
                    .x = x * 1.0f / (resolution-1) * side_length - (side_length / 2),
                    .y = y * 1.0f / (resolution-1) * side_length - (side_length / 2),
                    .z = z * 1.0f / (resolution-1) * side_length,
                };

                ++index;
            }
        }
    }
    printf("\n");

    return voxels;
}

mat4x4 generate_identitiy_4x4() {
    mat4x4 mat;
    memset(&mat, 0, sizeof(mat));
    mat.rows[0].v.x = 1;
    mat.rows[1].v.y = 1;
    mat.rows[2].v.z = 1;
    mat.rows[3].v.w = 1;

    return mat;
}

mat4x4 generate_look_at_mat(v3 eye, v3 center, v3 up) {
    v3 f = normalize(eye - center);
    v3 s = normalize(cross(f, up));
    v3 u = cross(s, f);

    mat4x4 result = generate_identitiy_4x4();
    result.rows[0].cols[0] = s.x;
    result.rows[0].cols[1] = s.y;
    result.rows[0].cols[2] = s.z;
    result.rows[1].cols[0] = u.x;
    result.rows[1].cols[1] = u.y;
    result.rows[1].cols[2] = u.z;
    result.rows[2].cols[0] =-f.x;
    result.rows[2].cols[1] =-f.y;
    result.rows[2].cols[2] =-f.z;
    result.rows[0].cols[3] =-dot(s, eye);
    result.rows[1].cols[3] =-dot(u, eye);
    result.rows[2].cols[3] = dot(f, eye);

    return result;
}

mat4x4 perspectiveRH_ZO(f32 fovy, f32 aspect, f32 zNear, f32 zFar) {
    // assert(abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

    f32 const tanHalfFovy = tan(fovy / 2.0f);

    mat4x4 Result;
    memset(&Result, 0, sizeof(Result));

    Result.rows[0].cols[0] = 1.0 / (aspect * tanHalfFovy);
    Result.rows[1].cols[1] = 1.0 / (tanHalfFovy);
    Result.rows[2].cols[2] = zFar / (zNear - zFar);
    Result.rows[3].cols[2] = -1.0;
    Result.rows[2].cols[3] = -(zFar * zNear) / (zFar - zNear);
    return Result;
}

mat3x3 generate_z_rot_mat(f32 angle) {
    mat3x3 mat;
    memset(&mat, 0, sizeof(mat));

    mat.rows[0].cols[0] =  cos(angle);
    mat.rows[0].cols[1] = -sin(angle);
    mat.rows[1].cols[0] =  sin(angle);
    mat.rows[1].cols[1] =  cos(angle);

    mat.rows[2].cols[2] = 1;

    return mat;

}

mat4x4 generate_extrinsic_mat(f32 offset_horiz, f32 offset_vert, f32 obj_rotation) {
    obj_rotation = -obj_rotation * M_PI / 180.0f;
    v3 eye    {offset_horiz, 0, offset_vert};
    v3 center {0, 0, 0};
    v3 up     {0, 0, 1};

    mat3x3 rot_z = generate_z_rot_mat(obj_rotation);
    eye = rot_z * eye;


    return generate_look_at_mat(eye, center, up);
}

mat4x4 to_4x4(mat3x3 m) {
    mat4x4 result;
    memset(&result, 0, sizeof(result));

    result.rows[0].cols[0] = m.rows[0].cols[0];
    result.rows[0].cols[1] = m.rows[0].cols[1];
    result.rows[0].cols[2] = m.rows[0].cols[2];

    result.rows[1].cols[0] = m.rows[1].cols[0];
    result.rows[1].cols[1] = m.rows[1].cols[1];
    result.rows[1].cols[2] = m.rows[1].cols[2];

    result.rows[2].cols[0] = m.rows[2].cols[0];
    result.rows[2].cols[1] = m.rows[2].cols[1];
    result.rows[2].cols[2] = m.rows[2].cols[2];

    result.rows[3].cols[3] = 1;

    return result;
}

v4 to_v4(v3 v) {
    return {
        v.x, v.y, v.z, 1
    };
}

v3 to_v3(v4 v) {
    return {
        v.x, v.y, v.z
    };
}


v3 project_voxel_to_screen_space(v3 pos, mat4x4 extrinsic, mat4x4 intrinsic) {

    v4 our_intrinsic[3];
    our_intrinsic[0] = {intrinsic.rows[0].cols[0], 0, 0, 0};
    our_intrinsic[1] = {0, intrinsic.rows[1].cols[1], 0, 0};
    our_intrinsic[2] = {0, 0, 1, 0};

    v4 our_temp[3];
    our_temp[0] = 
          { dot(our_intrinsic[0], 
                 {extrinsic.rows[0].cols[0],
                  extrinsic.rows[1].cols[0],
                  extrinsic.rows[2].cols[0],
                  extrinsic.rows[3].cols[0]}),
            dot(our_intrinsic[0], 
                 {extrinsic.rows[0].cols[1],
                  extrinsic.rows[1].cols[1],
                  extrinsic.rows[2].cols[1],
                  extrinsic.rows[3].cols[1]}),
            dot(our_intrinsic[0], 
                 {extrinsic.rows[0].cols[2],
                  extrinsic.rows[1].cols[2],
                  extrinsic.rows[2].cols[2],
                  extrinsic.rows[3].cols[2]}),
            dot(our_intrinsic[0], 
                 {extrinsic.rows[0].cols[3],
                  extrinsic.rows[1].cols[3],
                  extrinsic.rows[2].cols[3],
                  extrinsic.rows[3].cols[3]}),
          };
    our_temp[1] = 
          { dot(our_intrinsic[1], 
                 {extrinsic.rows[0].cols[0],
                  extrinsic.rows[1].cols[0],
                  extrinsic.rows[2].cols[0],
                  extrinsic.rows[3].cols[0]}),
            dot(our_intrinsic[1], 
                 {extrinsic.rows[0].cols[1],
                  extrinsic.rows[1].cols[1],
                  extrinsic.rows[2].cols[1],
                  extrinsic.rows[3].cols[1]}),
            dot(our_intrinsic[1], 
                 {extrinsic.rows[0].cols[2],
                  extrinsic.rows[1].cols[2],
                  extrinsic.rows[2].cols[2],
                  extrinsic.rows[3].cols[2]}),
            dot(our_intrinsic[1], 
                 {extrinsic.rows[0].cols[3],
                  extrinsic.rows[1].cols[3],
                  extrinsic.rows[2].cols[3],
                  extrinsic.rows[3].cols[3]}),
          };
    our_temp[2] = 
          { dot(our_intrinsic[2], 
                 {extrinsic.rows[0].cols[0],
                  extrinsic.rows[1].cols[0],
                  extrinsic.rows[2].cols[0],
                  extrinsic.rows[3].cols[0]}),
            dot(our_intrinsic[2], 
                 {extrinsic.rows[0].cols[1],
                  extrinsic.rows[1].cols[1],
                  extrinsic.rows[2].cols[1],
                  extrinsic.rows[3].cols[1]}),
            dot(our_intrinsic[2], 
                 {extrinsic.rows[0].cols[2],
                  extrinsic.rows[1].cols[2],
                  extrinsic.rows[2].cols[2],
                  extrinsic.rows[3].cols[2]}),
            dot(our_intrinsic[2], 
                 {extrinsic.rows[0].cols[3],
                  extrinsic.rows[1].cols[3],
                  extrinsic.rows[2].cols[3],
                  extrinsic.rows[3].cols[3]}),
          };

    v3 intermediate = 
       // to_v3(
          // intrinsic * 
          // (extrinsic * to_v4(pos));
       // );
    {
        dot(our_temp[0], to_v4(pos)),
        dot(our_temp[1], to_v4(pos)),
        dot(our_temp[2], to_v4(pos))
    };
    
    
    intermediate = (-1.0f /intermediate.z) * intermediate;

    return intermediate;

}

void print(mat4x4 m) {
    printf(" % 5.3f % 5.3f % 5.3f % 5.3f\n", m.rows[0].cols[0], m.rows[0].cols[1], m.rows[0].cols[2], m.rows[0].cols[3]);
    printf(" % 5.3f % 5.3f % 5.3f % 5.3f\n", m.rows[1].cols[0], m.rows[1].cols[1], m.rows[1].cols[2], m.rows[1].cols[3]);
    printf(" % 5.3f % 5.3f % 5.3f % 5.3f\n", m.rows[2].cols[0], m.rows[2].cols[1], m.rows[2].cols[2], m.rows[2].cols[3]);
    printf(" % 5.3f % 5.3f % 5.3f % 5.3f\n", m.rows[3].cols[0], m.rows[3].cols[1], m.rows[3].cols[2], m.rows[3].cols[3]);
}


int main(int argc, char *argv[]) {
    u32 res = 3;
    f32 sl = 0.098;

    Voxel* voxels = generate_voxel_grid(res, sl);

    for (u32 z = 0; z < res; ++z) {
        for (u32 y = 0; y < res; ++y) {
            for (u32 x = 0; x < res; ++x) {
                Voxel v = voxels[IDX3D(x,y,z, res)];
                printf("Voxel: % 3.2f % 3.2f % 3.2f\n",
                       v.position.x,
                       v.position.y,
                       v.position.z);                
            }
        }
    }

    mat4x4 extrinsic = generate_extrinsic_mat(0.62, 0.57, 0);
    print(extrinsic);


    int image_width = 11;
    int image_height = 12;
    int n;
    Pixel* pixels = (Pixel*)stbi_load("marker.jpg", &image_width, &image_height, &n, 0);

    f32 aspect = 1.0f * image_width / image_height;
    printf("image size: %d %d\n", image_width, image_height);

    mat4x4 intrinsic = perspectiveRH_ZO(0.3503711, aspect, 0.3, 200);
    print(intrinsic);


    for (u32 z = 0; z < res; ++z) {
        for (u32 y = 0; y < res; ++y) {
            for (u32 x = 0; x < res; ++x) {
                Voxel v = voxels[IDX3D(x,y,z, res)];
                v3 p = project_voxel_to_screen_space(v.position, extrinsic, intrinsic);

                // printf("Projected Voxel: % 3.2f % 3.2f % 3.2f\n",
                //        p.x,  p.y,  p.z);

                int p_x = (p.x + 1.0f) / 2.0f * image_width;
                int p_y = (p.y + 1.0f) / 2.0f * image_height;

                // printf("image spce coords: %d %d\n", p_x, p_y);

                int thickness = 10;

                for (int j = p_y - thickness/2; j < p_y+thickness/2; ++j) {
                    for (int i = p_x - thickness/2; i < p_x+thickness/2; ++i) {
                        if (j >= 0 && j < image_height && i >= 0 && i < image_width) {
                            pixels[j * image_width + i].r = 255;
                            pixels[j * image_width + i].g = 0;                        
                            pixels[j * image_width + i].b = 255;    
                        }                        
                    }
                }
            }
        }
    }

    stbi_write_png("marker_yes.png", image_width, image_height, 3, pixels, 0);

    return 0;
}
