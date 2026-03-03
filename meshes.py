from pathlib import Path
import trimesh
import json
import shutil

CREATE_MESHES = True
CREATE_COMPONENTS_AS_ROBOTS = False # used for visualization

ROBOT_NAME = "LeCAN_James"

assets_dir = Path("src/assets")
robot_dir = assets_dir / f"Robot_{ROBOT_NAME}"

config_file = list(robot_dir.glob("config.json"))[0]
config_data = json.loads(config_file.read_text(encoding="utf-8"))
components_data = config_data["components"]

model_index = 0
for file in robot_dir.iterdir():
    if "model_" in file.name:
        mesh = trimesh.load_mesh(file)
        hull = mesh.convex_hull

        if CREATE_MESHES:
            meshes_dir = Path("src/main/resources/meshes")
            meshes_file = meshes_dir / f"{file.stem}.obj"
            hull.export(meshes_file)
        
        if CREATE_COMPONENTS_AS_ROBOTS:
            model_dir = assets_dir / f"Robot_{ROBOT_NAME}_{model_index}"
            model_dir.mkdir(exist_ok=True)
            model_file = model_dir / "model.glb"
            shutil.copyfile(file, model_file)
            
            model_config_file = model_dir / "config.json"
            model_json_data = {
                "name": f"{config_data["name"]} {model_index}",
                "isFTC": False,
                "rotations": components_data[model_index]["zeroedRotations"]
            }
            with model_config_file.open("w", encoding="utf-8") as f:
                json.dump(model_json_data, f, indent=2)

        print(file.stem)
        model_index += 1

print("DONE")