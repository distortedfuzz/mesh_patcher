#ifndef POLYSCOPE_TRIAL_PATCH_UI_H
#define POLYSCOPE_TRIAL_PATCH_UI_H
#include "imgui.h"
#include "polyscope/polyscope.h"
#include "mesh.h"

int patch_index_input = 0;
int patch_index = 0;

void patch_callback(int patch_count, std::vector<mesh> &patch_meshes,
                mesh &main_mesh, std::vector<patch> &valid_patches) {

    ImGui::PushItemWidth(100);
    ImGui::Text("Total Patches: %d", patch_count);

    if(patch_index_input > patch_count - 1){
        patch_index_input = patch_count - 1;
    }

    if(patch_index_input < 0){
        patch_index_input = 0;
    }

    ImGui::InputInt("Patch Index", &patch_index_input);

    if (ImGui::Button("Get Patch")) {
        polyscope::removeStructure("path0", false);
        polyscope::removeStructure("path1", false);
        polyscope::removeStructure("path2", false);
        polyscope::removeStructure("path3", false);
        main_mesh.displayed_path_count = 0;

        polyscope::removeStructure("new_patch" + std::to_string(patch_index), false);
        patch_index = patch_index_input;

        patch_meshes[patch_index].display_mesh();
        main_mesh.display_path(valid_patches[patch_index].edges[0]);
        main_mesh.display_path(valid_patches[patch_index].edges[1]);
        main_mesh.display_path(valid_patches[patch_index].edges[2]);
        main_mesh.display_path(valid_patches[patch_index].edges[3]);


    }

    ImGui::PopItemWidth();
}

#endif
