#include "mesh.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "glm/vec3.hpp"
#include "deps/fiboheap.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <algorithm>


void mesh::read_off(const std::string &path){

    std::ifstream off_file(path);

    if(!off_file){
        std::cerr<<"Could not find .off file!"<<std::endl;
        return;
    }

    std::string format;
    off_file >> format;
    if(format != "OFF"){
        std::cerr<<"The file is not in .off format!"<<std::endl;
        return;
    }

    size_t vertex_count, face_count, edge_count;

    off_file >> vertex_count >> face_count >> edge_count;

    vertices.resize(vertex_count);

    for(size_t i = 0; i < vertex_count; i++){
        std::array<float, 3> new_coords{0,0,0};
        off_file >> new_coords[0] >> new_coords[1] >> new_coords[2];

        auto *new_vertex = new vertex();
        new_vertex->coords = new_coords;
        new_vertex->vertex_id = int(i);

        vertices[i] = new_vertex;
    }

    for(size_t i = 0; i < face_count; i++){
        size_t primitive_edge_count;
        off_file >> primitive_edge_count;

        auto *new_triangle = new triangle();

        off_file >> new_triangle->v1 >> new_triangle->v2 >> new_triangle->v3;
        new_triangle->triangle_id = int(i);

        triangles.push_back(new_triangle);

        //add neighbor triangles
        vertices[new_triangle->v1]->neighbor_triangles.push_back(int(i));
        vertices[new_triangle->v2]->neighbor_triangles.push_back(int(i));
        vertices[new_triangle->v3]->neighbor_triangles.push_back(int(i));

        //create new edges
        bool exists1 = false, exists2 = false, exists3 = false;

        for(auto &check_edge_index:vertices[new_triangle->v1]->neighbor_edges) {

            edge *check_edge = edges[check_edge_index];
            if ((check_edge->v1 == new_triangle->v1 && check_edge->v2 == new_triangle->v2) ||
                (check_edge->v2 == new_triangle->v1 && check_edge->v1 == new_triangle->v2)) {

                exists1 = true;

            }
        }

        for(auto &check_edge_index:vertices[new_triangle->v2]->neighbor_edges) {

            edge *check_edge = edges[check_edge_index];
            if ((check_edge->v1 == new_triangle->v2 && check_edge->v2 == new_triangle->v3) ||
                (check_edge->v2 == new_triangle->v2 && check_edge->v1 == new_triangle->v3)) {

                exists2 = true;

            }
        }

        for(auto &check_edge_index:vertices[new_triangle->v1]->neighbor_edges) {

            edge *check_edge = edges[check_edge_index];
            if((check_edge->v1 == new_triangle->v1 && check_edge->v2 == new_triangle->v3) ||
               (check_edge->v2 == new_triangle->v1 && check_edge->v1 == new_triangle->v3)){

                exists3 = true;

            }

        }

        if(!exists1){
            auto *new_edge = new edge();
            new_edge->edge_id = int(edges.size());
            new_edge->v1 = new_triangle->v1;
            new_edge->v2 = new_triangle->v2;

            glm::vec3 v1_coords{vertices[new_triangle->v1]->coords[0],
                                vertices[new_triangle->v1]->coords[1],
                                vertices[new_triangle->v1]->coords[2]};

            glm::vec3 v2_coords{vertices[new_triangle->v2]->coords[0],
                                vertices[new_triangle->v2]->coords[1],
                                vertices[new_triangle->v2]->coords[2]};

            new_edge->length = glm::distance(v1_coords, v2_coords);

            edges.push_back(new_edge);

            vertices[new_triangle->v1]->neighbor_edges.push_back(new_edge->edge_id);
            vertices[new_triangle->v2]->neighbor_edges.push_back(new_edge->edge_id);

            vertices[new_triangle->v1]->neighbor_vertices.push_back(new_triangle->v2);
            vertices[new_triangle->v2]->neighbor_vertices.push_back(new_triangle->v1);

        }

        if(!exists2){
            auto *new_edge = new edge();
            new_edge->edge_id = int(edges.size());
            new_edge->v1 = new_triangle->v2;
            new_edge->v2 = new_triangle->v3;

            glm::vec3 v2_coords{vertices[new_triangle->v2]->coords[0],
                                vertices[new_triangle->v2]->coords[1],
                                vertices[new_triangle->v2]->coords[2]};

            glm::vec3 v3_coords{vertices[new_triangle->v3]->coords[0],
                                vertices[new_triangle->v3]->coords[1],
                                vertices[new_triangle->v3]->coords[2]};

            new_edge->length = glm::distance(v2_coords, v3_coords);

            edges.push_back(new_edge);

            vertices[new_triangle->v2]->neighbor_edges.push_back(new_edge->edge_id);
            vertices[new_triangle->v3]->neighbor_edges.push_back(new_edge->edge_id);

            vertices[new_triangle->v2]->neighbor_vertices.push_back(new_triangle->v3);
            vertices[new_triangle->v3]->neighbor_vertices.push_back(new_triangle->v2);
        }

        if(!exists3){
            auto *new_edge = new edge();
            new_edge->edge_id = int(edges.size());
            new_edge->v1 = new_triangle->v1;
            new_edge->v2 = new_triangle->v3;

            glm::vec3 v1_coords{vertices[new_triangle->v1]->coords[0],
                                vertices[new_triangle->v1]->coords[1],
                                vertices[new_triangle->v1]->coords[2]};

            glm::vec3 v3_coords{vertices[new_triangle->v3]->coords[0],
                                vertices[new_triangle->v3]->coords[1],
                                vertices[new_triangle->v3]->coords[2]};

            new_edge->length = glm::distance(v1_coords, v3_coords);

            edges.push_back(new_edge);

            vertices[new_triangle->v1]->neighbor_edges.push_back(new_edge->edge_id);
            vertices[new_triangle->v3]->neighbor_edges.push_back(new_edge->edge_id);

            vertices[new_triangle->v1]->neighbor_vertices.push_back(new_triangle->v3);
            vertices[new_triangle->v3]->neighbor_vertices.push_back(new_triangle->v1);
        }

    }

    this->mesh_name = path;

}


void mesh::display_mesh(){

    std::vector<std::array<float, 3>> vertex_coordinates;

    vertex_coordinates.reserve(vertices.size());
    for(auto &vertex: vertices){
        vertex_coordinates.push_back(vertex->coords);
    }

    std::vector<std::array<int, 3>> triangle_indices;

    triangle_indices.reserve(triangles.size());
    for(auto &triangle: triangles){
        std::array<int, 3> triangle_vertices{triangle->v1, triangle->v2, triangle->v3};
        triangle_indices.push_back(triangle_vertices);
    }

    polyscope::registerSurfaceMesh(this->mesh_name,
                                   vertex_coordinates,
                                   triangle_indices);
}


void mesh::display_path(std::vector<int> &path){

    std::vector<std::array<float, 3>> nodes;
    for(auto &vert: path){

        std::array<float, 3> new_coords{vertices[vert]->coords[0],
                                        vertices[vert]->coords[1],
                                        vertices[vert]->coords[2]};

        nodes.push_back(new_coords);


    }

    std::vector<std::array<int, 2>> path_edges;

    for(int i = 0; i < path.size() - 1; i++){

        std::array<int, 2> new_edge{i, i+1};
        path_edges.push_back(new_edge);

    }

    polyscope::registerCurveNetwork("path" + std::to_string(this->displayed_path_count), nodes, path_edges);
    this->displayed_path_count++;

}


void mesh::display_samples(std::vector<int> &samples,float radius){

    std::vector<std::array<float, 3>> sample_coordinates;

    for(auto &sample: samples){

        std::array<float, 3> coordinate = vertices[sample]->coords;
        sample_coordinates.push_back(coordinate);

    }

    polyscope::PointCloud* sample_cloud = polyscope::registerPointCloud("samples" +
                                                                        std::to_string(this->displayed_sample_set_count),
                                                                        sample_coordinates);

    sample_cloud->setPointRadius(radius);
    this->displayed_sample_set_count++;

}

void mesh::display_coordinates(std::vector<std::array<float, 3>> &coordinates){

    polyscope::PointCloud* sample_cloud = polyscope::registerPointCloud("samples" +
                                                                        std::to_string(this->displayed_sample_set_count),
                                                                        coordinates);

    sample_cloud->setPointRadius(0.003);
    this->displayed_sample_set_count++;

}


void mesh::dijkstra_array(int start_index, bool rerun){

    if((!rerun) && (!vertices[start_index]->dijkstra_distances.empty())){
        return;
    }

    std::vector<bool> visited(vertices.size(), false);
    vertices[start_index]->dijkstra_distances = std::vector<float>(vertices.size(),INFINITY);
    vertices[start_index]->dijkstra_distances[start_index] = 0.0f;
    vertices[start_index]->dijkstra_previous_node = -1;


    for(int i = 0; i < vertices.size(); i++){

        float min_distance = INFINITY;
        int min_id = -1;

        for(int vert = 0; vert < vertices.size(); vert++){

            if(!visited[vert] && vertices[start_index]->dijkstra_distances[vert] < min_distance){
                min_distance = vertices[start_index]->dijkstra_distances[vert];
                min_id = vert;
            }

        }

        if(min_id == -1){
            break;
        }

        visited[min_id] = true;

        for(auto &neighbor_vert: vertices[min_id]->neighbor_vertices){

            if(!visited[neighbor_vert]){

                float edge_length = glm::distance(glm::vec3 {vertices[min_id]->coords[0],
                                                             vertices[min_id]->coords[1],
                                                             vertices[min_id]->coords[2]},
                                                  glm::vec3 {vertices[neighbor_vert]->coords[0],
                                                             vertices[neighbor_vert]->coords[1],
                                                             vertices[neighbor_vert]->coords[2]});

                if(vertices[start_index]->dijkstra_distances[min_id] + edge_length <
                   vertices[start_index]->dijkstra_distances[neighbor_vert]){

                    vertices[neighbor_vert]->dijkstra_previous_node = min_id;
                    vertices[start_index]->dijkstra_distances[neighbor_vert] = edge_length +
                            vertices[start_index]->dijkstra_distances[min_id];

                }
            }
        }
    }
}


void mesh::dijkstra_heap(int start_index, bool rerun){

    if((!rerun) && (!vertices[start_index]->dijkstra_distances.empty())){
        return;
    }

    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> min_heap;

    std::vector<bool> visited(vertices.size(), false);
    vertices[start_index]->dijkstra_distances = std::vector<float>(vertices.size(),INFINITY);
    vertices[start_index]->dijkstra_distances[start_index] = 0.0f;
    vertices[start_index]->dijkstra_previous_node = -1;

    std::pair<float, int> start_node{0.0f, start_index};
    min_heap.push(start_node);

    while(!min_heap.empty()){
        std::pair<float, int> current_node = min_heap.top();
        min_heap.pop();
        int min_id = current_node.second;

        visited[min_id] = true;

        for(auto &neighbor_vert: vertices[min_id]->neighbor_vertices){

            if(!visited[neighbor_vert]){

                float edge_length = glm::distance(glm::vec3 {vertices[min_id]->coords[0],
                                                             vertices[min_id]->coords[1],
                                                             vertices[min_id]->coords[2]},
                                                  glm::vec3 {vertices[neighbor_vert]->coords[0],
                                                             vertices[neighbor_vert]->coords[1],
                                                             vertices[neighbor_vert]->coords[2]});

                if(vertices[start_index]->dijkstra_distances[min_id] + edge_length <
                   vertices[start_index]->dijkstra_distances[neighbor_vert]){

                    vertices[neighbor_vert]->dijkstra_previous_node = min_id;
                    vertices[start_index]->dijkstra_distances[neighbor_vert] = edge_length +
                                                                               vertices[start_index]->dijkstra_distances[min_id];

                    std::pair<float, int> new_node{vertices[start_index]->dijkstra_distances[neighbor_vert], neighbor_vert};
                    min_heap.push(new_node);
                }

            }
        }
    }
}


void mesh::dijkstra_fib_heap(int start_index, bool rerun){

    if((!rerun) && (!vertices[start_index]->dijkstra_distances.empty())){
        return;
    }

    FibHeap<float> fib_heap;

    std::vector<bool> visited(vertices.size(), false);
    vertices[start_index]->dijkstra_distances = std::vector<float>(vertices.size(),INFINITY);
    vertices[start_index]->dijkstra_distances[start_index] = 0.0f;
    vertices[start_index]->dijkstra_previous_node = -1;


    std::vector<FibHeap<float>::FibNode*> nodes(vertices.size(), nullptr);

    auto* start_node = new FibHeap<float>::FibNode(0.0f, start_index);
    fib_heap.insert(start_node);
    nodes[start_index] = start_node;


    while(!fib_heap.empty()){
        FibHeap<float>::FibNode* min_node = fib_heap.minimum();
        int min_id = min_node->payload;

        fib_heap.pop();
        visited[min_id] = true;

        for(auto &neighbor_vert: vertices[min_id]->neighbor_vertices){

            if(!visited[neighbor_vert]){

                float edge_length = glm::distance(glm::vec3 {vertices[min_id]->coords[0],
                                                             vertices[min_id]->coords[1],
                                                             vertices[min_id]->coords[2]},
                                                  glm::vec3 {vertices[neighbor_vert]->coords[0],
                                                             vertices[neighbor_vert]->coords[1],
                                                             vertices[neighbor_vert]->coords[2]});

                if(vertices[start_index]->dijkstra_distances[min_id] + edge_length <
                   vertices[start_index]->dijkstra_distances[neighbor_vert]){

                    vertices[neighbor_vert]->dijkstra_previous_node = min_id;
                    vertices[start_index]->dijkstra_distances[neighbor_vert] = edge_length +
                                                                               vertices[start_index]->dijkstra_distances[min_id];

                    if(nodes[neighbor_vert] == nullptr){

                        auto* neighbor_node = new FibHeap<float>::FibNode(edge_length +
                                                                          vertices[start_index]->dijkstra_distances[min_id],
                                                                          neighbor_vert);
                        fib_heap.insert(neighbor_node);
                        nodes[neighbor_vert] = neighbor_node;
                    }else{

                        fib_heap.decrease_key(nodes[neighbor_vert], edge_length +
                                                                    vertices[start_index]->dijkstra_distances[min_id]);
                    }
                }

            }
        }
    }
}


std::vector<int> mesh::create_dijkstra_path(int target){

    std::vector<int> path;
    path.push_back(target);

    int current_index = target;

    while(vertices[current_index]->dijkstra_previous_node != -1){

        path.push_back(vertices[current_index]->dijkstra_previous_node);
        current_index = vertices[current_index]->dijkstra_previous_node;

    }

    return path;

}

std::vector<int> mesh::create_dijkstra_path_farthest(int start_index){

    int index = -1;
    float max_dist = 0.0f;
    for(int i = 0; i < vertices[start_index]->dijkstra_distances.size(); i++){

        if(vertices[start_index]->dijkstra_distances[i] > max_dist){
            index = i;
            max_dist = vertices[start_index]->dijkstra_distances[i];
        }
    }

    std::vector<int> path;
    path.push_back(index);

    int current_index = index;

    while(vertices[current_index]->dijkstra_previous_node != -1){

        path.push_back(vertices[current_index]->dijkstra_previous_node);
        current_index = vertices[current_index]->dijkstra_previous_node;

    }

    return path;

}

float mesh::get_path_length(std::vector<int> &path){

    float distance = 0.0f;

    for(int i = 0; i < path.size() - 1; i++){

        distance += get_length_between_vertices(path[i], path[i + 1]);

    }

    return distance;

}

float mesh::get_length_between_vertices(int vert1, int vert2){


        return glm::distance(glm::vec3 {vertices[vert1]->coords[0],
                                             vertices[vert1]->coords[1],
                                             vertices[vert1]->coords[2]},
                                  glm::vec3 {vertices[vert2]->coords[0],
                                             vertices[vert2]->coords[1],
                                             vertices[vert2]->coords[2]});


}


std::vector<int> mesh::farthest_point_sample(int sample_count){

    dijkstra_fib_heap(0);

    int max_index = -1;
    float max_dist = 0.0f;
    for(int i = 0; i < vertices[0]->dijkstra_distances.size(); i++){
        if(vertices[0]->dijkstra_distances[i] > max_dist){
            max_dist = vertices[0]->dijkstra_distances[i];
            max_index = i;
        }
    }

    std::vector<int> samples;
    samples.push_back(max_index);

    for(int i = 0; i < sample_count - 1; i++){

        dijkstra_fib_heap(samples[i]);
        std::vector<float> vertex_association(vertices.size(), INFINITY);

        for(auto &existing_sample: samples){

            for(int j = 0; j < vertex_association.size(); j++){

                if(vertices[existing_sample]->dijkstra_distances[j] < vertex_association[j]){
                    vertex_association[j] = vertices[existing_sample]->dijkstra_distances[j];
                }

            }

        }

        int max_fps_index = -1;
        float max_fps_distance = 0.0f;
        for(int j = 0; j < vertex_association.size(); j++){
            if(vertex_association[j] > max_fps_distance){
                max_fps_distance = vertex_association[j];
                max_fps_index = j;
            }
        }

        samples.push_back(max_fps_index);

    }

    return samples;
}


std::vector<std::vector<int>> mesh::geodesic_paths_between_samples(std::vector<int> &samples){

    std::vector<std::vector<int>> all_paths;

    for(int i = 0; i < samples.size(); i++){

        dijkstra_fib_heap(samples[i], true);

        for(int j = i + 1; j < samples.size(); j++){

            std::vector<int> new_path = create_dijkstra_path(samples[j]);
            all_paths.push_back(new_path);

        }

    }

    return all_paths;

}


std::vector<std::vector<int>> mesh::create_patch_edges(std::array<std::vector<int>, 4> &path_quadruple){

    std::vector<int> intersection_vector;
    for(int i = 0; i < path_quadruple.size(); i++){

        for(int j = i + 1; j < path_quadruple.size(); j++){

            for(auto &original_path_node: path_quadruple[i]){

                for(auto &check_path_node: path_quadruple[j]){

                    if(check_path_node == original_path_node){

                        bool already_included = false;
                        for(auto &existing_node: intersection_vector){

                            if(check_path_node == existing_node){
                                already_included = true;
                            }

                        }

                        if(!already_included){
                            intersection_vector.push_back(check_path_node);
                        }

                    }

                }

            }

        }

    }

    std::vector<std::vector<int>> all_edges;
    if(intersection_vector.size() != 4){
        return all_edges;
    }

    for(int i = 0; i < 4; i++){

        std::vector<int> current_path = path_quadruple[i];

        std::vector<int> new_edge;
        bool edge_started = false;
        for(auto &node: current_path){

            bool is_intersection = (std::find(intersection_vector.begin(),
                                              intersection_vector.end(),
                                              node) != intersection_vector.end());

            if (edge_started){
                new_edge.push_back(node);
            }

            if (is_intersection){
                if (edge_started){
                    all_edges.push_back(new_edge);
                    break;
                }else{
                    new_edge.push_back(node);
                    edge_started = true;
                }
            }

        }

    }

    for(const auto & all_edge : all_edges){
        if(all_edge.size() < 3){
            return std::vector<std::vector<int>>{};
        }
    }

    return all_edges;

}


std::vector<patch> mesh::get_valid_patches(std::vector<std::vector<int>> &geodesic_paths){

    std::vector<patch> patches;

    for(int i = 0; i < geodesic_paths.size(); i++){
        for(int j = i + 1; j < geodesic_paths.size(); j++){
            for(int k = j + 1; k < geodesic_paths.size(); k++){
                for(int l = k + 1; l < geodesic_paths.size(); l++){

                    std::array<std::vector<int>, 4> path_quadruple;
                    path_quadruple[0] = geodesic_paths[i];
                    path_quadruple[1] = geodesic_paths[j];
                    path_quadruple[2] = geodesic_paths[k];
                    path_quadruple[3] = geodesic_paths[l];

                    std::vector<std::vector<int>> patch_edges = create_patch_edges(path_quadruple);

                    if(patch_edges.size() != 4){
                        continue;
                    }

                    int distinct_end_count = 0;
                    std::vector<std::pair<int, int>> existing_end_counts;
                    for(auto &edge: patch_edges){

                        bool found1 = false;
                        bool found2 = false;
                        for(auto &end: existing_end_counts){
                            if(end.second == edge.front()){
                                end.first ++;
                                found1 = true;
                            }

                            if(end.second == edge.back()){
                                end.first ++;
                                found2 = true;
                            }
                        }

                        if(!found1){
                            std::pair<int, int> new_end{1, edge.front()};
                            existing_end_counts.push_back(new_end);
                        }

                        if(!found2){
                            std::pair<int, int> new_end{1, edge.back()};
                            existing_end_counts.push_back(new_end);
                        }

                    }

                    if(existing_end_counts.size() != 4){
                        continue;
                    }

                    bool inadequate = false;
                    for(auto &end: existing_end_counts){
                        if(end.first != 2){
                            inadequate = true;
                            break;
                        }
                    }

                    if(inadequate){
                        continue;
                    }

                    patch new_patch;
                    new_patch.edges = patch_edges;
                    patches.push_back(new_patch);

                }
            }
        }
    }

    return patches;

}


void mesh::uniform_sample_patch_edges(patch &input_patch){

    std::pair<int, int> column_pair{-1, -1};
    std::pair<int, int> row_pair{-1, -1};

    column_pair.first = 0;

    for(int i = 1; i < 4; i++){

        if((input_patch.edges[i].front() != input_patch.edges[0].front()) &&
            (input_patch.edges[i].back() != input_patch.edges[0].back()) &&
            (input_patch.edges[i].front() != input_patch.edges[0].back()) &&
            (input_patch.edges[i].back() != input_patch.edges[0].front()) ){

            column_pair.second = i;

        }

    }

    for(int i = 0; i < 4; i++){

        if(i != column_pair.first && i != column_pair.second){
            if(row_pair.first == -1){
                row_pair.first = i;
            }else{
                row_pair.second = i;
            }
        }

    }

    if(column_pair.first == -1 || column_pair.second == -1 ||
        row_pair.first == -1 || row_pair.second == -1){
        return;
    }

    if(int(input_patch.edges[column_pair.first].size()) < int(input_patch.edges[column_pair.second].size())){
        input_patch.column_vertex_count = int(input_patch.edges[column_pair.first].size());
        input_patch.column1_samples = input_patch.edges[column_pair.first];

        float longer_edge_length = get_path_length(input_patch.edges[column_pair.second]);
        float step_size = longer_edge_length / float(input_patch.column_vertex_count);

        input_patch.column2_samples.push_back(input_patch.edges[column_pair.second][0]);

        float accumulated_step = step_size;
        float accumulated_distance = 0.0f;

        for(int i = 0; i < input_patch.edges[column_pair.second].size() - 1; i++){

            float path_edge_length = get_length_between_vertices(input_patch.edges[column_pair.second][i],
                                                                 input_patch.edges[column_pair.second][i + 1]);

            accumulated_distance += path_edge_length;

            if(accumulated_distance >= accumulated_step){
                input_patch.column2_samples.push_back(input_patch.edges[column_pair.second][i + 1]);
                accumulated_step += step_size;
            }

        }

        if(input_patch.column2_samples.size() > input_patch.column1_samples.size()){
            int diff = int(input_patch.column2_samples.size()) - int(input_patch.column1_samples.size());

            for(int i = 0; i < diff + 1; i++){
                input_patch.column2_samples.pop_back();
            }

            input_patch.column2_samples.push_back(input_patch.edges[column_pair.second].back());
        }

        input_patch.column2_samples.pop_back();
        input_patch.column2_samples.push_back(input_patch.edges[column_pair.second].back());


    }else{
        input_patch.column_vertex_count = int(input_patch.edges[column_pair.second].size());
        input_patch.column1_samples = input_patch.edges[column_pair.second];

        float longer_edge_length = get_path_length(input_patch.edges[column_pair.first]);
        float step_size = longer_edge_length / float(input_patch.column_vertex_count);

        input_patch.column2_samples.push_back(input_patch.edges[column_pair.first][0]);

        float accumulated_step = step_size;
        float accumulated_distance = 0.0f;

        for(int i = 0; i < input_patch.edges[column_pair.first].size() - 1; i++){

            float path_edge_length = get_length_between_vertices(input_patch.edges[column_pair.first][i],
                                                                 input_patch.edges[column_pair.first][i + 1]);

            accumulated_distance += path_edge_length;

            if(accumulated_distance >= accumulated_step){
                input_patch.column2_samples.push_back(input_patch.edges[column_pair.first][i + 1]);
                accumulated_step += step_size;
            }

        }

        if(input_patch.column2_samples.size() > input_patch.column1_samples.size()){
            int diff = int(input_patch.column2_samples.size()) - int(input_patch.column1_samples.size());

            for(int i = 0; i < diff + 1; i++){
                input_patch.column2_samples.pop_back();
            }

            input_patch.column2_samples.push_back(input_patch.edges[column_pair.first].back());
        }

        input_patch.column2_samples.pop_back();
        input_patch.column2_samples.push_back(input_patch.edges[column_pair.first].back());

    }


    if(int(input_patch.edges[row_pair.first].size()) < int(input_patch.edges[row_pair.second].size())){
        input_patch.row_vertex_count = int(input_patch.edges[row_pair.first].size());
        input_patch.row1_samples = input_patch.edges[row_pair.first];

        float longer_edge_length = get_path_length(input_patch.edges[row_pair.second]);
        float step_size = longer_edge_length / float(input_patch.row_vertex_count);

        input_patch.row2_samples.push_back(input_patch.edges[row_pair.second][0]);

        float accumulated_step = step_size;
        float accumulated_distance = 0.0f;

        for(int i = 0; i < input_patch.edges[row_pair.second].size() - 1; i++){

            float path_edge_length = get_length_between_vertices(input_patch.edges[row_pair.second][i],
                                                                 input_patch.edges[row_pair.second][i + 1]);

            accumulated_distance += path_edge_length;

            if(accumulated_distance >= accumulated_step){
                input_patch.row2_samples.push_back(input_patch.edges[row_pair.second][i + 1]);
                accumulated_step += step_size;
            }

        }

        if(input_patch.row2_samples.size() > input_patch.row1_samples.size()){
            int diff = int(input_patch.row2_samples.size()) - int(input_patch.row1_samples.size());

            for(int i = 0; i < diff + 1; i++){
                input_patch.row2_samples.pop_back();
            }

            input_patch.row2_samples.push_back(input_patch.edges[row_pair.second].back());
        }

        input_patch.row2_samples.pop_back();
        input_patch.row2_samples.push_back(input_patch.edges[row_pair.second].back());

    }else{
        input_patch.row_vertex_count = int(input_patch.edges[row_pair.second].size());
        input_patch.row1_samples = input_patch.edges[row_pair.second];

        float longer_edge_length = get_path_length(input_patch.edges[row_pair.first]);
        float step_size = longer_edge_length / float(input_patch.row_vertex_count);

        input_patch.row2_samples.push_back(input_patch.edges[row_pair.first][0]);

        float accumulated_step = step_size;
        float accumulated_distance = 0.0f;

        for(int i = 0; i < input_patch.edges[row_pair.first].size() - 1; i++){

            float path_edge_length = get_length_between_vertices(input_patch.edges[row_pair.first][i],
                                                                 input_patch.edges[row_pair.first][i + 1]);

            accumulated_distance += path_edge_length;

            if(accumulated_distance >= accumulated_step){
                input_patch.row2_samples.push_back(input_patch.edges[row_pair.first][i + 1]);
                accumulated_step += step_size;
            }

        }

        if(input_patch.row2_samples.size() > input_patch.row1_samples.size()){
            int diff = int(input_patch.row2_samples.size()) - int(input_patch.row1_samples.size());

            for(int i = 0; i < diff + 1; i++){
                input_patch.row2_samples.pop_back();
            }

            input_patch.row2_samples.push_back(input_patch.edges[row_pair.first].back());
        }

        input_patch.row2_samples.pop_back();
        input_patch.row2_samples.push_back(input_patch.edges[row_pair.first].back());

    }

}


void mesh::align_edges(patch &input_patch){

    if((input_patch.column1_samples.back() == input_patch.row1_samples.front()) ||
        (input_patch.column1_samples.back() == input_patch.row1_samples.back())){
        std::reverse(input_patch.column1_samples.begin(), input_patch.column1_samples.end());
    }

    if((input_patch.column2_samples.back() == input_patch.row1_samples.front()) ||
       (input_patch.column2_samples.back() == input_patch.row1_samples.back())){
        std::reverse(input_patch.column2_samples.begin(), input_patch.column2_samples.end());
    }

    if(input_patch.row1_samples.front() != input_patch.column1_samples.front()){
        std::reverse(input_patch.row1_samples.begin(), input_patch.row1_samples.end());
    }

    if(input_patch.row2_samples.front() != input_patch.column1_samples.back()){
        std::reverse(input_patch.row2_samples.begin(), input_patch.row2_samples.end());
    }
}


glm::vec3 mesh::catmull_rom(glm::vec3 &p0, glm::vec3 &p1, glm::vec3 &p2, glm::vec3 &p3, float t){
    float t2 = t * t;
    float t3 = t2 * t;
    return 0.5f * ((2.0f * p1) +
                   (-p0 + p2) * t +
                   (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                   (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}


glm::vec3 mesh::evaluate_spline(std::vector<glm::vec3> &control_points, float weight){

    int count = control_points.size();
    if (count == 0){
        return glm::vec3(0.0f);
    }else if (count == 1){
        return control_points[0];
    }

    float spline_ratio = weight * (count - 1);

    int vertex_index = std::min(int(spline_ratio), count - 2);

    float local_spline_t = spline_ratio - vertex_index;

    glm::vec3 p0{0.0f,0.0f,0.0f};
    if(vertex_index == 0){
        p0 = control_points[0];
    }else{
        p0 = control_points[vertex_index - 1];
    }

    glm::vec3 p1 = control_points[vertex_index];
    glm::vec3 p2 = control_points[vertex_index + 1];

    glm::vec3 p3{0.0f,0.0f,0.0f};

    if(vertex_index + 2 > count){
        p3 = control_points[count - 1];
    }else{
        p3 = control_points[vertex_index + 2];
    }

    return catmull_rom(p0, p1, p2, p3, local_spline_t);
}


void mesh::generate_patch_vertices(patch &input_patch){

    if (input_patch.row1_samples.empty() || input_patch.row2_samples.empty() ||
        input_patch.column1_samples.empty() || input_patch.column2_samples.empty()) {
        return;
    }

    glm::vec3 top_left = glm::vec3(vertices[input_patch.row1_samples.front()]->coords[0],
                                   vertices[input_patch.row1_samples.front()]->coords[1],
                                   vertices[input_patch.row1_samples.front()]->coords[2]);

    glm::vec3 top_right = glm::vec3(vertices[input_patch.row1_samples.back()]->coords[0],
                                    vertices[input_patch.row1_samples.back()]->coords[1],
                                    vertices[input_patch.row1_samples.back()]->coords[2]);

    glm::vec3 bottom_left = glm::vec3(vertices[input_patch.row2_samples.front()]->coords[0],
                                      vertices[input_patch.row2_samples.front()]->coords[1],
                                      vertices[input_patch.row2_samples.front()]->coords[2]);

    glm::vec3 bottom_right = glm::vec3(vertices[input_patch.row2_samples.back()]->coords[0],
                                       vertices[input_patch.row2_samples.back()]->coords[1],
                                       vertices[input_patch.row2_samples.back()]->coords[2]);

    input_patch.vertices.clear();

    std::vector<glm::vec3> left_boundary_coords, right_boundary_coords;
    std::vector<glm::vec3> top_boundary_coords, bottom_boundary_coords;

    for (auto sample : input_patch.column1_samples){
        left_boundary_coords.push_back(glm::vec3(vertices[sample]->coords[0],
                                                 vertices[sample]->coords[1],
                                                 vertices[sample]->coords[2]));
    }

    for (auto sample : input_patch.column2_samples){
        right_boundary_coords.push_back(glm::vec3(vertices[sample]->coords[0],
                                                 vertices[sample]->coords[1],
                                                 vertices[sample]->coords[2]));
    }

    for (auto sample : input_patch.row1_samples){
        top_boundary_coords.push_back(glm::vec3(vertices[sample]->coords[0],
                                                  vertices[sample]->coords[1],
                                                  vertices[sample]->coords[2]));
    }

    for (auto sample : input_patch.row2_samples){
        bottom_boundary_coords.push_back(glm::vec3(vertices[sample]->coords[0],
                                                vertices[sample]->coords[1],
                                                vertices[sample]->coords[2]));
    }


    for (int i = 0; i < input_patch.column_vertex_count; i++) {
        float row_weight = float(i) / float(input_patch.column_vertex_count - 1);

        std::vector<std::array<float, 3>> current_row;
        glm::vec3 left_spline = evaluate_spline(left_boundary_coords, row_weight);
        glm::vec3 right_spline = evaluate_spline(right_boundary_coords, row_weight);

        for (int j = 0; j < input_patch.row_vertex_count; j++) {

            float column_weight = float(j) / float(input_patch.row_vertex_count - 1);

            glm::vec3 top_spline = evaluate_spline(top_boundary_coords, column_weight);
            glm::vec3 bottom_spline = evaluate_spline(bottom_boundary_coords, column_weight);

            glm::vec3 top_bottom_linear_interpolation = glm::mix(top_spline, bottom_spline, row_weight);
            glm::vec3 left_right_linear_interpolation = glm::mix(left_spline, right_spline, column_weight);

            glm::vec3 bilinear_interpolation= glm::mix(glm::mix(top_left, top_right, column_weight),
                                                       glm::mix(bottom_left, bottom_right, column_weight),
                                                       row_weight);
            glm::vec3 new_vertex = top_bottom_linear_interpolation + left_right_linear_interpolation
                            - bilinear_interpolation;

            current_row.push_back(std::array<float, 3>{new_vertex.x,
                                                       new_vertex.y,
                                                       new_vertex.z });
        }
        input_patch.vertices.push_back(current_row);
    }
}


void mesh::generate_patch_vertices_hard(patch &input_patch){

    //get first row
    std::vector<std::array<float, 3>> start_row;

    for(auto &node: input_patch.row1_samples){
        std::array<float, 3> new_node_coords = vertices[node]->coords;
        start_row.push_back(new_node_coords);
    }

    input_patch.vertices.push_back(start_row);


    //middle rows

    for(int i = 1; i < input_patch.column_vertex_count - 1; i++){

        std::vector<std::array<float, 3>> mid_row;

        std::array<float, 3> row_start_vert = vertices[input_patch.column1_samples[i]]->coords;
        mid_row.push_back(row_start_vert);

        std::array<float, 3> row_end_vert = vertices[input_patch.column2_samples[i]]->coords;

        glm::vec3 start_coord{row_start_vert[0],
                              row_start_vert[1],
                              row_start_vert[2]};

        glm::vec3 end_coord{row_end_vert[0],
                            row_end_vert[1],
                            row_end_vert[2]};

        glm::vec3 row_normal = end_coord - start_coord;
        float row_length = glm::length(row_normal);
        float step_size = row_length / float(input_patch.row_vertex_count - 1);

        row_normal = glm::normalize(row_normal);
        float cum_step = step_size;

        for(int j = 1; j < input_patch.row_vertex_count - 1; j++){

            glm::vec3 new_coord = start_coord + (row_normal * cum_step);
            std::array<float, 3> new_arr{new_coord[0],
                                         new_coord[1],
                                         new_coord[2]};

            mid_row.push_back(new_arr);
            cum_step += step_size;

        }

        mid_row.push_back(row_end_vert);

        input_patch.vertices.push_back(mid_row);
    }


    //get final row
    std::vector<std::array<float, 3>> final_row;

    for(auto &node: input_patch.row2_samples){
        std::array<float, 3> new_node_coords = vertices[node]->coords;
        final_row.push_back(new_node_coords);
    }

    input_patch.vertices.push_back(final_row);

}


void mesh::generate_patch_vertices_hard_shift(patch &input_patch){


    //get first row
    std::vector<std::array<float, 3>> start_row;

    for(auto &node: input_patch.row1_samples){
        std::array<float, 3> new_node_coords = vertices[node]->coords;
        start_row.push_back(new_node_coords);
    }

    input_patch.vertices.push_back(start_row);


    //middle rows

    for(int i = 1; i < input_patch.column_vertex_count - 1; i++){

        float row_weight = float(i) / float(input_patch.column_vertex_count - 1);

        std::vector<std::array<float, 3>> mid_row;

        std::array<float, 3> row_start_vert = vertices[input_patch.column1_samples[i]]->coords;
        mid_row.push_back(row_start_vert);

        std::array<float, 3> row_end_vert = vertices[input_patch.column2_samples[i]]->coords;

        glm::vec3 start_coord{row_start_vert[0],
                              row_start_vert[1],
                              row_start_vert[2]};

        glm::vec3 end_coord{row_end_vert[0],
                            row_end_vert[1],
                            row_end_vert[2]};

        glm::vec3 row_normal = end_coord - start_coord;
        float row_length = glm::length(row_normal);
        float step_size = row_length / float(input_patch.row_vertex_count - 1);

        row_normal = glm::normalize(row_normal);
        float cum_step = step_size;

        for(int j = 1; j < input_patch.row_vertex_count - 1; j++){

            glm::vec3 row1_node1{vertices[input_patch.row1_samples[j-1]]->coords[0],
                                 vertices[input_patch.row1_samples[j-1]]->coords[1],
                                 vertices[input_patch.row1_samples[j-1]]->coords[2]};
            glm::vec3 row1_node2{vertices[input_patch.row1_samples[j]]->coords[0],
                                 vertices[input_patch.row1_samples[j]]->coords[1],
                                 vertices[input_patch.row1_samples[j]]->coords[2]};

            glm::vec3 row1_vector = row1_node2 - row1_node1;

            glm::vec3 row2_node1{vertices[input_patch.row2_samples[j-1]]->coords[0],
                                 vertices[input_patch.row2_samples[j-1]]->coords[1],
                                 vertices[input_patch.row2_samples[j-1]]->coords[2]};
            glm::vec3 row2_node2{vertices[input_patch.row2_samples[j]]->coords[0],
                                 vertices[input_patch.row2_samples[j]]->coords[1],
                                 vertices[input_patch.row2_samples[j]]->coords[2]};

            glm::vec3 row2_vector = row2_node2 - row2_node1;

            glm::vec3 shift_vec = (1-row_weight) * row1_vector + row_weight * row2_vector;

            glm::vec3 new_coord = start_coord + (row_normal * cum_step) + shift_vec;
            std::array<float, 3> new_arr{new_coord[0],
                                         new_coord[1],
                                         new_coord[2]};

            mid_row.push_back(new_arr);
            cum_step += step_size;

        }

        mid_row.push_back(row_end_vert);

        input_patch.vertices.push_back(mid_row);
    }


    //get final row
    std::vector<std::array<float, 3>> final_row;

    for(auto &node: input_patch.row2_samples){
        std::array<float, 3> new_node_coords = vertices[node]->coords;
        final_row.push_back(new_node_coords);
    }

    input_patch.vertices.push_back(final_row);

}


mesh mesh::meshify_patch(patch &input_patch, std::string name){

    mesh new_mesh;

    int vertex_id = 0;
    for(auto &row: input_patch.vertices){

        for(auto &vertex_coordinate: row){

            auto *new_vertex = new vertex();
            new_vertex->coords = vertex_coordinate;
            new_vertex->vertex_id = vertex_id;
            new_mesh.vertices.push_back(new_vertex);

            vertex_id++;
        }

    }

    int column_length = input_patch.vertices.size();
    int row_length = input_patch.vertices[0].size();
    int triangle_id = 0;

    for(int i = 0; i < column_length - 1; i++){

        for(int j = 1; j < row_length; j++){
            auto *new_triangle1 = new triangle();

            new_triangle1->v1 = (i * row_length) + j - 1;
            new_triangle1->v2 = (i * row_length) + j;
            new_triangle1->v3 = ((i+1) * row_length) + j - 1;
            new_triangle1->triangle_id = triangle_id;
            triangle_id++;

            auto *new_triangle2 = new triangle();

            new_triangle2->v1 = (i * row_length) + j;
            new_triangle2->v2 = ((i+1) * row_length) + j;
            new_triangle2->v3 = ((i+1) * row_length) + j - 1;
            new_triangle2->triangle_id = triangle_id;
            triangle_id++;

            new_mesh.triangles.push_back(new_triangle1);
            new_mesh.triangles.push_back(new_triangle2);

        }
    }

    new_mesh.mesh_name = name;

    return new_mesh;

}

mesh mesh::meshify_patch_all(patch &input_patch, std::string &name){

    mesh new_mesh;
    new_mesh.mesh_name = name;

    int vertex_id = 0;
    std::vector<int> last_patch_row;
    std::vector<int> last_patch_column;
    //row
    for(int i = 0; i < input_patch.vertices.size(); i++){

        //each node in a row
        for(int j = 0; j < input_patch.vertices[i].size(); j++){

            if(i == input_patch.vertices.size() - 2){
                last_patch_row.push_back(vertex_id);
            }

            if(j == input_patch.vertices[i].size() - 2){
                last_patch_column.push_back(vertex_id);
            }

            auto *new_vertex = new vertex();
            new_vertex->coords = input_patch.vertices[i][j];
            new_vertex->vertex_id = vertex_id;
            new_mesh.vertices.push_back(new_vertex);

            vertex_id++;
        }

    }

    int column_length = input_patch.vertices.size();
    int row_length = input_patch.vertices[0].size();
    int triangle_id = 0;


    for(int i = 0; i < column_length - 2; i++){

        for(int j = 1; j < row_length - 1; j++){
            auto *new_triangle1 = new triangle();

            new_triangle1->v1 = (i * row_length) + j - 1;
            new_triangle1->v2 = (i * row_length) + j;
            new_triangle1->v3 = ((i+1) * row_length) + j - 1;
            new_triangle1->triangle_id = triangle_id;
            triangle_id++;

            auto *new_triangle2 = new triangle();

            new_triangle2->v1 = (i * row_length) + j;
            new_triangle2->v2 = ((i+1) * row_length) + j;
            new_triangle2->v3 = ((i+1) * row_length) + j - 1;
            new_triangle2->triangle_id = triangle_id;
            triangle_id++;

            new_mesh.triangles.push_back(new_triangle1);
            new_mesh.triangles.push_back(new_triangle2);

        }
    }

    //find which edges are row2 and column2
    //add reverses
    int row2_edge_index = -1;
    int column2_edge_index = -1;
    if(input_patch.edges[0].front() == input_patch.column2_samples.front() &&
        input_patch.edges[0].back() == input_patch.column2_samples.back()){

        column2_edge_index = 0;

    }else if(input_patch.edges[0].front() == input_patch.column2_samples.back() &&
        input_patch.edges[0].back() == input_patch.column2_samples.front()){

        column2_edge_index = 0;
        std::reverse(input_patch.edges[column2_edge_index].begin(), input_patch.edges[column2_edge_index].end());

    }else if((input_patch.edges[1].front() == input_patch.column2_samples.front() &&
        input_patch.edges[1].back() == input_patch.column2_samples.back())){

        column2_edge_index = 1;

    }else if(input_patch.edges[1].front() == input_patch.column2_samples.back() &&
        input_patch.edges[1].back() == input_patch.column2_samples.front()){

        column2_edge_index = 1;
        std::reverse(input_patch.edges[column2_edge_index].begin(), input_patch.edges[column2_edge_index].end());

    }else if((input_patch.edges[2].front() == input_patch.column2_samples.front() &&
              input_patch.edges[2].back() == input_patch.column2_samples.back())){

        column2_edge_index = 2;

    }else if(input_patch.edges[2].front() == input_patch.column2_samples.back() &&
             input_patch.edges[2].back() == input_patch.column2_samples.front()){

        column2_edge_index = 2;
        std::reverse(input_patch.edges[column2_edge_index].begin(), input_patch.edges[column2_edge_index].end());

    }else if((input_patch.edges[3].front() == input_patch.column2_samples.front() &&
              input_patch.edges[3].back() == input_patch.column2_samples.back())){

        column2_edge_index = 3;

    }else if(input_patch.edges[3].front() == input_patch.column2_samples.back() &&
             input_patch.edges[3].back() == input_patch.column2_samples.front()){

        column2_edge_index = 3;
        std::reverse(input_patch.edges[column2_edge_index].begin(), input_patch.edges[column2_edge_index].end());

    }

    if(input_patch.edges[0].front() == input_patch.row2_samples.front() &&
       input_patch.edges[0].back() == input_patch.row2_samples.back()){

        row2_edge_index = 0;

    }else if(input_patch.edges[0].front() == input_patch.row2_samples.back() &&
             input_patch.edges[0].back() == input_patch.row2_samples.front()){

        row2_edge_index = 0;
        std::reverse(input_patch.edges[row2_edge_index].begin(), input_patch.edges[row2_edge_index].end());

    }else if((input_patch.edges[1].front() == input_patch.row2_samples.front() &&
              input_patch.edges[1].back() == input_patch.row2_samples.back())){

        row2_edge_index = 1;

    }else if(input_patch.edges[1].front() == input_patch.row2_samples.back() &&
             input_patch.edges[1].back() == input_patch.row2_samples.front()){

        row2_edge_index = 1;
        std::reverse(input_patch.edges[row2_edge_index].begin(), input_patch.edges[row2_edge_index].end());

    }else if((input_patch.edges[2].front() == input_patch.row2_samples.front() &&
              input_patch.edges[2].back() == input_patch.row2_samples.back())){

        row2_edge_index = 2;

    }else if(input_patch.edges[2].front() == input_patch.row2_samples.back() &&
             input_patch.edges[2].back() == input_patch.row2_samples.front()){

        row2_edge_index = 2;
        std::reverse(input_patch.edges[row2_edge_index].begin(), input_patch.edges[row2_edge_index].end());

    }else if((input_patch.edges[3].front() == input_patch.row2_samples.front() &&
              input_patch.edges[3].back() == input_patch.row2_samples.back())){

        row2_edge_index = 3;

    }else if(input_patch.edges[3].front() == input_patch.row2_samples.back() &&
             input_patch.edges[3].back() == input_patch.row2_samples.front()){

        row2_edge_index = 3;
        std::reverse(input_patch.edges[row2_edge_index].begin(), input_patch.edges[row2_edge_index].end());

    }

    std::vector<int> long_row = input_patch.edges[row2_edge_index];
    std::vector<int> long_column = input_patch.edges[column2_edge_index];


    int v3_index = 0;
    for(int i = 1; i < long_row.size(); i++){
        if(i == 1){
            auto *first_vertex = new vertex();
            first_vertex->coords = this->vertices[long_row[0]]->coords;
            first_vertex->vertex_id = vertex_id;
            new_mesh.vertices.push_back(first_vertex);

            vertex_id++;

            auto *second_vertex = new vertex();
            second_vertex->coords = this->vertices[long_row[1]]->coords;
            second_vertex->vertex_id = vertex_id;
            new_mesh.vertices.push_back(second_vertex);

            vertex_id++;

            auto *new_triangle = new triangle();
            new_triangle->v1 = first_vertex->vertex_id;
            new_triangle->v2 = last_patch_row[0];
            new_triangle->v3 = second_vertex->vertex_id;

            v3_index = 0;
            new_triangle->triangle_id = triangle_id;

            new_mesh.triangles.push_back(new_triangle);

            triangle_id++;

        }else if(i > 1 && i < long_row.size() - 1){

            int last_added_vert_index = vertex_id - 1;

            auto *new_vertex = new vertex();
            new_vertex->coords = this->vertices[long_row[i]]->coords;
            new_vertex->vertex_id = vertex_id;
            new_mesh.vertices.push_back(new_vertex);

            vertex_id++;

            auto *new_triangle = new triangle();

            // decide new_triangle->v3
            glm::vec3 current_vert_coord{new_vertex->coords[0],
                                         new_vertex->coords[1],
                                         new_vertex->coords[2]};

            int closest_index = -1;
            float closest_distance = INFINITY;
            for(int k = 0; k < last_patch_row.size() - 1; k++){
                glm::vec3 check_vert_coord{new_mesh.vertices[last_patch_row[k]]->coords[0],
                                           new_mesh.vertices[last_patch_row[k]]->coords[1],
                                           new_mesh.vertices[last_patch_row[k]]->coords[2]};

                float dist = glm::distance(current_vert_coord, check_vert_coord);
                if(dist < closest_distance){
                    closest_distance = dist;
                    closest_index = k;
                }
            }

            if(closest_index == -1){
                closest_index = last_patch_row.size() - 2;
            }


            new_triangle->v1 = last_added_vert_index;
            new_triangle->v2 = last_patch_row[closest_index];
            new_triangle->v3 = new_vertex->vertex_id;

            new_triangle->triangle_id = triangle_id;

            new_mesh.triangles.push_back(new_triangle);

            int v3_difference = -1;
            if(closest_index != v3_index){
                v3_difference = closest_index - v3_index;
            }

            int v3_looper1 = v3_index;

            for(int h = 0; h < v3_difference; h++){

                auto *new_v3_triangle1 = new triangle();

                new_v3_triangle1->v1 = last_added_vert_index;
                new_v3_triangle1->v2 = last_patch_row[v3_index + h];
                new_v3_triangle1->v3 = last_patch_row[v3_index + h + 1];

                new_v3_triangle1->triangle_id = triangle_id;
                new_mesh.triangles.push_back(new_v3_triangle1);
                triangle_id++;
            }

            v3_index = closest_index;

        }else if(i == long_row.size() - 1){
            int last_added_vert_index = vertex_id - 1;

            auto *new_vertex = new vertex();
            new_vertex->coords = this->vertices[long_row[i]]->coords;
            new_vertex->vertex_id = vertex_id;
            new_mesh.vertices.push_back(new_vertex);

            vertex_id++;

            auto *new_triangle = new triangle();

            // decide new_triangle->v3
            glm::vec3 current_vert_coord{new_vertex->coords[0],
                                         new_vertex->coords[1],
                                         new_vertex->coords[2]};

            int closest_index = -1;
            float closest_distance = INFINITY;
            for(int k = 0; k < last_patch_row.size() - 1; k++){
                glm::vec3 check_vert_coord{new_mesh.vertices[last_patch_row[k]]->coords[0],
                                           new_mesh.vertices[last_patch_row[k]]->coords[1],
                                           new_mesh.vertices[last_patch_row[k]]->coords[2]};

                float dist = glm::distance(current_vert_coord, check_vert_coord);
                if(dist < closest_distance){
                    closest_distance = dist;
                    closest_index = k;
                }
            }

            if(closest_index == -1){
                closest_index = last_patch_row.size() - 2;
            }


            new_triangle->v1 = last_added_vert_index;
            new_triangle->v2 = last_patch_row[closest_index];
            new_triangle->v3 = new_vertex->vertex_id;

            new_triangle->triangle_id = triangle_id;

            new_mesh.triangles.push_back(new_triangle);

            int v3_difference = -1;
            if(closest_index != v3_index){
                v3_difference = closest_index - v3_index;
            }

            int v3_looper1 = v3_index;

            for(int h = 0; h < v3_difference; h++){

                auto *new_v3_triangle1 = new triangle();

                new_v3_triangle1->v1 = new_vertex->vertex_id;
                new_v3_triangle1->v2 = last_patch_row[v3_index + h];
                new_v3_triangle1->v3 = last_patch_row[v3_index + h + 1];

                new_v3_triangle1->triangle_id = triangle_id;
                new_mesh.triangles.push_back(new_v3_triangle1);
                triangle_id++;
            }

            int last_diff = 0;
            if(closest_index != last_patch_row.size() - 1){
                last_diff = last_patch_row.size() - 2 - closest_index;
            }



            int current_last = closest_index;
            for(int l = 0; l < last_diff; l++){

                auto *new_v3_triangle1 = new triangle();

                new_v3_triangle1->v1 = new_vertex->vertex_id;
                new_v3_triangle1->v2 = last_patch_row[current_last];
                new_v3_triangle1->v3 = last_patch_row[current_last + 1];

                new_v3_triangle1->triangle_id = triangle_id;
                new_mesh.triangles.push_back(new_v3_triangle1);
                triangle_id++;

            }
        }
    }




    v3_index = 0;
    for(int i = 1; i < long_column.size(); i++){
        if(i == 1){
            auto *first_vertex = new vertex();
            first_vertex->coords = this->vertices[long_column[0]]->coords;
            first_vertex->vertex_id = vertex_id;
            new_mesh.vertices.push_back(first_vertex);

            vertex_id++;

            auto *second_vertex = new vertex();
            second_vertex->coords = this->vertices[long_column[1]]->coords;
            second_vertex->vertex_id = vertex_id;
            new_mesh.vertices.push_back(second_vertex);

            vertex_id++;

            auto *new_triangle = new triangle();
            new_triangle->v1 = first_vertex->vertex_id;
            new_triangle->v2 = second_vertex->vertex_id;
            new_triangle->v3 = last_patch_column[0];
            v3_index = 0;
            new_triangle->triangle_id = triangle_id;

            new_mesh.triangles.push_back(new_triangle);

            triangle_id++;

        }else if(i > 1 && i < long_column.size() - 1){

            int last_added_vert_index = vertex_id - 1;

            auto *new_vertex = new vertex();
            new_vertex->coords = this->vertices[long_column[i]]->coords;
            new_vertex->vertex_id = vertex_id;
            new_mesh.vertices.push_back(new_vertex);

            vertex_id++;

            auto *new_triangle = new triangle();
            new_triangle->v1 = last_added_vert_index;
            new_triangle->v2 = new_vertex->vertex_id;

            // decide new_triangle->v3
            glm::vec3 current_vert_coord{new_vertex->coords[0],
                                         new_vertex->coords[1],
                                         new_vertex->coords[2]};

            int closest_index = -1;
            float closest_distance = INFINITY;
            for(int k = 0; k < last_patch_column.size() - 1; k++){
                glm::vec3 check_vert_coord{new_mesh.vertices[last_patch_column[k]]->coords[0],
                                           new_mesh.vertices[last_patch_column[k]]->coords[1],
                                           new_mesh.vertices[last_patch_column[k]]->coords[2]};

                float dist = glm::distance(current_vert_coord, check_vert_coord);
                if(dist < closest_distance){
                    closest_distance = dist;
                    closest_index = k;
                }
            }

            if(closest_index == -1){
                closest_index = last_patch_column.size() - 2;
            }


            new_triangle->v3 = last_patch_column[closest_index];

            new_triangle->triangle_id = triangle_id;
            triangle_id++;

            new_mesh.triangles.push_back(new_triangle);

            int v3_difference = -1;
            if(closest_index != v3_index){
                v3_difference = closest_index - v3_index;
            }

            int v3_looper1 = v3_index;

            for(int h = 0; h < v3_difference; h++){

                auto *new_v3_triangle1 = new triangle();

                new_v3_triangle1->v1 = last_added_vert_index;
                new_v3_triangle1->v2 = last_patch_column[v3_index + h + 1];
                new_v3_triangle1->v3 = last_patch_column[v3_index + h];
                new_v3_triangle1->triangle_id = triangle_id;
                new_mesh.triangles.push_back(new_v3_triangle1);
                triangle_id++;
            }

            v3_index = closest_index;

        }else if(i == long_column.size() - 1){
            int last_added_vert_index = vertex_id - 1;

            auto *new_vertex = new vertex();
            new_vertex->coords = this->vertices[long_column[i]]->coords;
            new_vertex->vertex_id = vertex_id;
            new_mesh.vertices.push_back(new_vertex);

            vertex_id++;

            auto *new_triangle = new triangle();

            // decide new_triangle->v3
            glm::vec3 current_vert_coord{new_vertex->coords[0],
                                         new_vertex->coords[1],
                                         new_vertex->coords[2]};

            int closest_index = -1;
            float closest_distance = INFINITY;
            for(int k = 0; k < last_patch_column.size() - 1; k++){
                glm::vec3 check_vert_coord{new_mesh.vertices[last_patch_column[k]]->coords[0],
                                           new_mesh.vertices[last_patch_column[k]]->coords[1],
                                           new_mesh.vertices[last_patch_column[k]]->coords[2]};

                float dist = glm::distance(current_vert_coord, check_vert_coord);
                if(dist < closest_distance){
                    closest_distance = dist;
                    closest_index = k;
                }
            }

            if(closest_index == -1){
                closest_index = last_patch_column.size() - 2;
            }


            new_triangle->v1 = last_added_vert_index;
            new_triangle->v2 = new_vertex->vertex_id;
            new_triangle->v3 = last_patch_column[closest_index];

            new_triangle->triangle_id = triangle_id;

            new_mesh.triangles.push_back(new_triangle);

            int v3_difference = -1;
            if(closest_index != v3_index){
                v3_difference = closest_index - v3_index;
            }

            int v3_looper1 = v3_index;

            for(int h = 0; h < v3_difference; h++){

                auto *new_v3_triangle1 = new triangle();

                new_v3_triangle1->v1 = last_added_vert_index;
                new_v3_triangle1->v2 = last_patch_column[v3_index + h];
                new_v3_triangle1->v3 = last_patch_column[v3_index + h + 1];

                new_v3_triangle1->triangle_id = triangle_id;
                new_mesh.triangles.push_back(new_v3_triangle1);
                triangle_id++;
            }

            int last_diff = 0;
            if(closest_index != last_patch_column.size() - 1){
                last_diff = last_patch_column.size() - 2 - closest_index;
            }


            int current_last = closest_index;
            for(int l = 0; l < last_diff; l++){

                auto *new_v3_triangle1 = new triangle();

                new_v3_triangle1->v1 = new_vertex->vertex_id;
                new_v3_triangle1->v2 = last_patch_column[current_last];
                new_v3_triangle1->v3 = last_patch_column[current_last + 1];

                new_v3_triangle1->triangle_id = triangle_id;
                new_mesh.triangles.push_back(new_v3_triangle1);
                triangle_id++;

            }
        }
    }


    return new_mesh;
}


void mesh::shift_vertices(patch &input_patch){

    // Loop over rows (skipping first and last)
    for (int i = 1; i < input_patch.vertices.size() - 1; i++) {

        std::array<float, 3> start_coords = input_patch.vertices[i][0];

        // Get the distance to the start row by walking backwards
        float full_distance = 0.0f;
        glm::vec3 current_row_node{ start_coords[0], start_coords[1], start_coords[2] };

        for (int j = i; j >= 0; j--) {
            std::array<float, 3> current_coords = input_patch.vertices[j][0];
            glm::vec3 new_node{ current_coords[0], current_coords[1], current_coords[2] };

            float distance = glm::distance(new_node, current_row_node);
            full_distance += distance;
            current_row_node = new_node;
        }
        float distance_to_start = full_distance;

        // Get the distance to the end row by walking forwards
        full_distance = 0.0f;
        current_row_node = { start_coords[0], start_coords[1], start_coords[2] };

        for (int j = i; j < input_patch.vertices.size(); j++) {
            std::array<float, 3> current_coords = input_patch.vertices[j][0];
            glm::vec3 new_node{ current_coords[0], current_coords[1], current_coords[2] };

            float distance = glm::distance(new_node, current_row_node);
            full_distance += distance;
            current_row_node = new_node;
        }
        float distance_to_end = full_distance;

        float distance_ratio = distance_to_start / (distance_to_start + distance_to_end);

        // Iterate over inner vertices of row i (skipping first and last columns)
        for (int j = 1; j < input_patch.vertices[i].size() - 1; j++) {
            std::array<float, 3> current_node_coords = input_patch.vertices[i][j];
            glm::vec3 current_node{ current_node_coords[0], current_node_coords[1], current_node_coords[2] };

            // Compute the distance from the current node to the start of the row
            float full_node_distance = 0.0f;
            glm::vec3 node_current = current_node;
            for (int k = j; k >= 0; k--) {
                std::array<float, 3> coords = input_patch.vertices[i][k];
                glm::vec3 new_node{ coords[0], coords[1], coords[2] };

                float distance = glm::distance(new_node, node_current);
                full_node_distance += distance;
                node_current = new_node;
            }
            float node_distance_to_start = full_node_distance;

            // Compute the distance from the current node to the end of the row
            full_node_distance = 0.0f;
            node_current = glm::vec3{ current_node_coords[0], current_node_coords[1], current_node_coords[2] };
            for (int k = j; k < input_patch.vertices[i].size(); k++) {
                std::array<float, 3> coords = input_patch.vertices[i][k];
                glm::vec3 new_node{ coords[0], coords[1], coords[2] };

                float distance = glm::distance(new_node, node_current);
                full_node_distance += distance;
                node_current = new_node;
            }
            float node_distance_to_end = full_node_distance;

            float node_distance_ratio = node_distance_to_start / (node_distance_to_start + node_distance_to_end);
            float node_distance_difference = distance_ratio - node_distance_ratio;
            std::cout<<distance_ratio<<" "<<node_distance_ratio<<std::endl;

            // Reset the current node position
            current_node = glm::vec3{ current_node_coords[0], current_node_coords[1], current_node_coords[2] };

            std::array<float, 3> prev_node_coords = input_patch.vertices[i][j - 1];
            glm::vec3 prev_node{ prev_node_coords[0], prev_node_coords[1], prev_node_coords[2] };

            glm::vec3 node_vec = prev_node - current_node;
            node_vec = glm::normalize(node_vec);
            node_vec = node_distance_difference * 50.0f * node_vec;

            current_node += node_vec;
            std::array<float, 3> new_node_coords{ current_node[0], current_node[1], current_node[2] };

            input_patch.vertices[i][j] = new_node_coords;
        }
    }
}


void mesh::vector_shift_patch_generation(patch &input_patch){

    //get row1 and row2 vectors
    std::vector<glm::vec3> start_row_vectors;
    for(int i = 0; i < input_patch.row1_samples.size() - 1; i++){
        std::array<float, 3> node1_coords = vertices[input_patch.row1_samples[i]]->coords;
        glm::vec3 node1{node1_coords[0], node1_coords[1], node1_coords[2]};

        std::array<float, 3> node2_coords = vertices[input_patch.row1_samples[i + 1]]->coords;
        glm::vec3 node2{node2_coords[0], node2_coords[1], node2_coords[2]};

        glm::vec3 vec = node2 - node1;
        start_row_vectors.push_back(vec);
    }

    std::vector<glm::vec3> end_row_vectors;
    for(int i = 0; i < input_patch.row2_samples.size() - 1; i++){
        std::array<float, 3> node1_coords = vertices[input_patch.row2_samples[i]]->coords;
        glm::vec3 node1{node1_coords[0], node1_coords[1], node1_coords[2]};

        std::array<float, 3> node2_coords = vertices[input_patch.row2_samples[i + 1]]->coords;
        glm::vec3 node2{node2_coords[0], node2_coords[1], node2_coords[2]};

        glm::vec3 vec = node2 - node1;
        end_row_vectors.push_back(vec);
    }

    //get first row
    std::vector<std::array<float, 3>> start_row;

    for(auto &node: input_patch.row1_samples){
        std::array<float, 3> new_node_coords = vertices[node]->coords;
        start_row.push_back(new_node_coords);
    }

    input_patch.vertices.push_back(start_row);

    //create patch, middle rows
    for(int i = 1; i < input_patch.column_vertex_count - 1; i++){

        float row_ratio = float(i) / float(input_patch.column_vertex_count - 1);

        //get vector and distance between row start and end
        std::array<float, 3> row_start_coord = vertices[input_patch.column1_samples[i]]->coords;
        glm::vec3 row_start{row_start_coord[0],
                               row_start_coord[1],
                               row_start_coord[2]};

        std::array<float, 3> row_end_coord = vertices[input_patch.column2_samples[i]]->coords;
        glm::vec3 row_end{row_end_coord[0],
                            row_end_coord[1],
                            row_end_coord[2]};

        glm::vec3 full_row_vector = row_end - row_start;
        glm::vec3 adapted_row_vec = full_row_vector / float(input_patch.row_vertex_count - 1);

        std::vector<std::array<float, 3>> new_row;
        new_row.push_back(row_start_coord);

        glm::vec3 current_node = row_start;

        for(int j = 1; j < input_patch.row_vertex_count - 1; j++){

            glm::vec3 start_row_vector = start_row_vectors[j];
            glm::vec3 end_row_vector = end_row_vectors[j];

            glm::vec3 weighted_average_vec = ((1.0f - row_ratio) * start_row_vector) + ((row_ratio) * end_row_vector);

            float cos_theta = glm::dot(glm::normalize(weighted_average_vec), glm::normalize(adapted_row_vec));

            if(cos_theta < 1e-2){
                cos_theta = 1e-2;
            }

            float adapted_row_vec_length = glm::length(adapted_row_vec);

            float average_vec_length = adapted_row_vec_length / cos_theta;

            weighted_average_vec = glm::normalize(weighted_average_vec);

            weighted_average_vec = weighted_average_vec * average_vec_length;


            glm::vec3 new_node = current_node + weighted_average_vec;

            std::array<float, 3> new_node_element{new_node[0],
                                                  new_node[1],
                                                  new_node[2]};

            new_row.push_back(new_node_element);
            current_node = new_node;

        }

        new_row.push_back(row_end_coord);
        input_patch.vertices.push_back(new_row);

    }


    //get final row
    std::vector<std::array<float, 3>> final_row;

    for(auto &node: input_patch.row2_samples){
        std::array<float, 3> new_node_coords = vertices[node]->coords;
        final_row.push_back(new_node_coords);
    }

    input_patch.vertices.push_back(final_row);

}
