#ifndef POLYSCOPE_TRIAL_MESH_H
#define POLYSCOPE_TRIAL_MESH_H
#include "glm/vec3.hpp"
#include "vector"

struct vertex{
    std::array<float, 3> coords;
    int vertex_id;

    std::vector<int> neighbor_vertices;
    std::vector<int> neighbor_edges;
    std::vector<int> neighbor_triangles;

    int dijkstra_previous_node;
    std::vector<float> dijkstra_distances;
};

struct edge{
    int v1, v2;
    int edge_id;
    float length;
};

struct triangle{
    int v1, v2, v3;
    int triangle_id;
};

struct patch{
    std::vector<std::vector<int>> edges;
    std::vector<std::vector<std::array<float, 3>>> vertices;
    std::vector<int> row1_samples;
    std::vector<int> row2_samples;
    std::vector<int> column1_samples;
    std::vector<int> column2_samples;
    int column_vertex_count;
    int row_vertex_count;
};

class mesh {

private:
    std::vector<vertex*> vertices;
    std::vector<edge*> edges;
    std::vector<triangle*> triangles;
    int displayed_sample_set_count = 0;

public:
    //IO
    int displayed_path_count = 0;
    std::string mesh_name;
    void read_off(const std::string &path);
    void display_mesh();
    void display_path(std::vector<int> &path);
    void display_samples(std::vector<int> &samples, float radius = 0.02f);
    void display_coordinates(std::vector<std::array<float, 3>> &coordinates);

    //GEODESICS
    void dijkstra_array(int start_index, bool rerun = false);
    void dijkstra_heap(int start_index, bool rerun = false);
    void dijkstra_fib_heap(int start_index, bool rerun = false);
    std::vector<int> create_dijkstra_path(int target);
    std::vector<int> create_dijkstra_path_farthest(int start_index);
    float get_path_length(std::vector<int> &path);
    float get_length_between_vertices(int vert1, int vert2);

    //SAMPLING
    std::vector<int> farthest_point_sample(int sample_count);

    //PATCHING
    std::vector<std::vector<int>> geodesic_paths_between_samples(std::vector<int> &samples);
    std::vector<patch> get_valid_patches(std::vector<std::vector<int>> &geodesic_paths);
    std::vector<std::vector<int>> create_patch_edges(std::array<std::vector<int>, 4> &path_quadruple);

    void uniform_sample_patch_edges(patch &input_patch);
    static void align_edges(patch &input_patch);

    glm::vec3 catmull_rom(glm::vec3 &p0, glm::vec3 &p1, glm::vec3 &p2, glm::vec3 &p3, float t);
    glm::vec3 evaluate_spline(std::vector<glm::vec3>& controlPoints, float t);

    void generate_patch_vertices(patch &input_patch);
    void generate_patch_vertices_hard(patch &input_patch);
    void generate_patch_vertices_hard_shift(patch &input_patch);
    void shift_vertices(patch &input_patch);

    void vector_shift_patch_generation(patch &input_patch);
    mesh meshify_patch(patch &input_patch, std::string name);
    mesh meshify_patch_all(patch &input_patch, std::string &name);

};


#endif
