#include <iostream>
#include <atomic>
#include <thread>
#include "polyscope/polyscope.h"
#include "mesh.h"
#include "patch_ui.h"


int main(int argc, char* argv[]) {

    if(argc != 3){
        std::cerr<<"Please input the file name and the sample count!"<<std::endl;
    }


    polyscope::init();

    std::string model_path;
    model_path = argv[1];

    int sample_count = std::stoi(argv[2]);

    mesh new_mesh;
    new_mesh.read_off(model_path);

    std::vector<int> samples = new_mesh.farthest_point_sample(sample_count);
    new_mesh.display_samples(samples);

    std::vector<std::vector<int>> geodesic_paths = new_mesh.geodesic_paths_between_samples(samples);
    std::vector<patch> valid_patches = new_mesh.get_valid_patches(geodesic_paths);

    std::vector<mesh> patch_meshes(valid_patches.size());

    uint32_t processing_unit_count = std::thread::hardware_concurrency();
    if (processing_unit_count == 0) {
        processing_unit_count = 8;
    }

    std::vector<std::thread> processing_units;
    processing_units.reserve(processing_unit_count);

    std::atomic<int> cursor = 0;

    for (int i = 0; i < processing_unit_count; i++) {
        processing_units.push_back(std::thread([&]() {
            while (true) {
                int patch_id = cursor.fetch_add(1, std::memory_order_relaxed);

                if(patch_id > valid_patches.size() - 1){
                    break;
                }
                new_mesh.uniform_sample_patch_edges(valid_patches[patch_id]);

                if(!valid_patches[patch_id].row1_samples.empty()){
                    mesh::align_edges(valid_patches[patch_id]);
                    new_mesh.generate_patch_vertices(valid_patches[patch_id]);

                    std::string name = "new_patch" + std::to_string(patch_id);
                    mesh patch_mesh = new_mesh.meshify_patch_all(valid_patches[patch_id], name);


                    patch_meshes[patch_id] = patch_mesh;

                }
            }
        }));
    }

    for (auto &processing_unit: processing_units) {
        processing_unit.join();
    }

    patch_meshes[0].display_mesh();
    new_mesh.display_path(valid_patches[0].edges[0]);
    new_mesh.display_path(valid_patches[0].edges[1]);
    new_mesh.display_path(valid_patches[0].edges[2]);
    new_mesh.display_path(valid_patches[0].edges[3]);

    int patch_count = patch_meshes.size() - 1;
    polyscope::state::userCallback = [patch_count, &patch_meshes, &new_mesh, &valid_patches]() {
        patch_callback(patch_count, patch_meshes, new_mesh, valid_patches);
    };


    new_mesh.display_mesh();
    polyscope::show();

}
