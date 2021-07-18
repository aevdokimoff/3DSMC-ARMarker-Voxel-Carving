struct Vertex {
    Vec3d pos;
    Vec2d uv_coord;
    Vec3d normal;
};

struct Mesh {
    Array_List<Vertex> vertices;
    Array_List<u32>    indices;
};

struct Vertex_Fingerprint {
    // TODO(Felix): why are all these u64?
    u64 pos_i;
    u64 norm_i;
    u64 uv_i;
};


inline auto hm_objects_match(Vertex_Fingerprint a, Vertex_Fingerprint b) -> bool {
    return a.pos_i  == b.pos_i
        && a.uv_i   == b.uv_i
        && a.norm_i == b.norm_i;
}

auto hm_hash(Vertex_Fingerprint v) -> u64 {
    u32 h = 0;
    u32 highorder = h & 0xf8000000;     // extract high-order 5 bits from h
                                        // 0xf8000000 is the hexadecimal representation
                                        //   for the 32-bit number with the first five
                                        //   bits = 1 and the other bits = 0
    h = h << 5;                         // shift h left by 5 bits
    h = h ^ (highorder >> 27);          // move the highorder 5 bits to the low-order
                                        //   end and XOR into h
    h = h ^ (u32)v.pos_i;               // XOR h and ki
                                        //
    highorder = h & 0xf8000000;
    h = h << 5;
    h = h ^ (highorder >> 27);
    h = h ^ (u32)v.norm_i;

    highorder = h & 0xf8000000;
    h = h << 5;
    h = h ^ (highorder >> 27);
    h = h ^ (u32)v.uv_i;

    return h;
}

auto read_entire_file(const char* filename) -> Ftb_String {
    Ftb_String ret;
    ret.data = nullptr;
    FILE *fp = fopen(filename, "rb");
    if (fp) {
        /*k Go to the end of the file. */
        if (fseek(fp, 0L, SEEK_END) == 0) {
            /* Get the size of the file. */
            ret.length = ftell(fp) + 1;
            if (ret.length == 0) {
                fputs("Empty file", stderr);
                goto closeFile;
            }

            /* Go back to the start of the file. */
            if (fseek(fp, 0L, SEEK_SET) != 0) {
                fprintf(stderr, "Error reading file");
                goto closeFile;
            }

            /* Allocate our buffer to that size. */
            ret.data = (char*)calloc(ret.length, sizeof(char));

            /* Read the entire file into memory. */
            ret.length = fread(ret.data, sizeof(char), ret.length, fp);

            ret.data[ret.length] = '\0';
            if (ferror(fp) != 0) {
                fprintf(stderr, "Error reading file");
                return {};
            }
        }
    closeFile:
        fclose(fp);
    } else {
        fprintf(stderr, "Cannot read file: %s\n", filename);
        return {};
    }

    return ret;
    /* Don't forget to call free() later! */
}

Mesh* load_obj(const char* path)  {

    Ftb_String obj_str = read_entire_file(path);
    defer_free(obj_str.data);

    Mesh* result = (Mesh*)malloc(sizeof(Mesh));
    result->vertices.alloc();
    result->indices.alloc();

    Auto_Array_List<f32> positions(512);
    Auto_Array_List<f32> normals(512);
    Auto_Array_List<f32> uvs(512);
    Auto_Array_List<Vertex_Fingerprint> fprints(512);

    char* cursor = obj_str.data;

    auto eat_line = [&]() {
        while (!(*cursor == '\n' || *cursor == '\r' || *cursor == '\0'))
            ++cursor;
    };

    auto eat_whitespace = [&]() {
        while (*cursor == ' '  || *cursor == '\n' ||
               *cursor == '\t' || *cursor == '\r') {
            ++cursor;
        }
    };

    auto eat_until_relevant = [&]() {
        char* old_read_pos;
        do {
            old_read_pos = cursor;
            eat_whitespace();
            if (*cursor == '#') eat_line(); // comment
            if (*cursor == 'o') eat_line(); // object name
            if (*cursor == 'l') eat_line(); // poly lines
            if (*cursor == 'u') eat_line(); // ???
            if (*cursor == 's') eat_line(); // smooth shading
            if (*cursor == 'm') eat_line(); // material
        } while(cursor != old_read_pos);
    };


    auto read_float = [&]() -> f32 {
        eat_whitespace();
        u64 res = 0;
        f32 negation = 1.0f;
        if (*cursor == '-') {
            negation = -1.0f;
            ++cursor;
        }
        while (*cursor >= '0' && *cursor <= '9') {
            res = res*10 + (*cursor - '0');
            ++cursor;
        }
        if (*cursor == '.') {
            int div = 1;
            ++cursor;
            while (*cursor >= '0' && *cursor <= '9') {
                res = res*10 + (*cursor - '0');
                div *= 10;
                ++cursor;
            }
            return res / (negation * div);
        }
        return negation*res;
    };

    auto read_int = [&]() -> u32 {
        eat_whitespace();
        u64 res = 0;
        s32 negation = 1;
        if (*cursor == '-') {
            negation = -1;
            ++cursor;
        }
        while (*cursor >= '0' && *cursor <= '9') {
            res = res*10 + (*cursor - '0');
            ++cursor;
        }
        return negation*(u32)res;
    };
    {
        char* eof = obj_str.data+obj_str.length;
        while (true) {
            eat_until_relevant();
            f32 x, y, z, u, v;
            if (cursor == eof)
                break;
            if (*cursor == 'v') {
                ++cursor;
                if (*cursor == ' ') {
                    // vertex pos
                    ++cursor;
                    x = read_float();
                    y = read_float();
                    z = read_float();
                    positions.extend({x, y, z});
                } else if (*cursor == 'n') {
                    // vertex normal
                    ++cursor;
                    x = read_float();
                    y = read_float();
                    z = read_float();
                    normals.extend({x, y, z});
                } else if (*cursor == 't') {
                    // vertex texture corrds
                    ++cursor;
                    u = read_float();
                    v = read_float();
                    v = 1 - v; // NOTE(Felix): Invert v, because in blender v goes up
                    uvs.extend({u, v});
                } else {
                    fprintf(stderr, "unknown marker \"v%c\" at %u", *cursor, (u32)(cursor-obj_str.data));
                    return nullptr;
                }
            } else if (*cursor == 'f') {
                // ZoneScopedN("read f");
                ++cursor;
                Vertex_Fingerprint vfp;
                for (u32 i = 0; i < 3; ++i) {
                    vfp.pos_i  = read_int();
                    ++cursor; // overstep slash
                    vfp.uv_i   = read_int();
                    ++cursor; // overstep slash
                    vfp.norm_i = read_int();

                    --vfp.pos_i;  // NOTE(Felix): the indices in the obj file start at 1
                    --vfp.uv_i;   // NOTE(Felix): the indices in the obj file start at 1
                    --vfp.norm_i; // NOTE(Felix): the indices in the obj file start at 1

                    fprints.append(vfp);
                }
            } else {
                fprintf(stderr, "unknown marker \"%c\" (pos: %ld)", *cursor, cursor-obj_str.data);
                return nullptr;
            }
        }
    }
    Hash_Map<Vertex_Fingerprint, u32> vertex_fp_to_index;
    vertex_fp_to_index.alloc(fprints.count);
    defer { vertex_fp_to_index.dealloc(); };

    {
        for (auto vfp : fprints) {
            u32 index = vertex_fp_to_index.get_index_of_living_cell_if_it_exists(vfp, hm_hash(vfp));
            if (index != -1) {
                result->indices.append(vertex_fp_to_index.data[index].object);
            } else {
                Vertex v{
                    Vec3d(
                        positions[3 * (u32)vfp.pos_i + 0],
                        positions[3 * (u32)vfp.pos_i + 1],
                        positions[3 * (u32)vfp.pos_i + 2]),
                    Vec2d(
                        uvs[2 * (u32)vfp.uv_i + 0],
                        uvs[2 * (u32)vfp.uv_i + 1]),
                    Vec3d(
                        normals[3 * (u32)vfp.norm_i + 0],
                        normals[3 * (u32)vfp.norm_i + 1],
                        normals[3 * (u32)vfp.norm_i + 2])
                };
                u32 new_index = result->vertices.count;
                result->vertices.append(v);
                result->indices.append(new_index);
                vertex_fp_to_index.set_object(vfp, new_index);
            }
        }
    }

    return result;
}
