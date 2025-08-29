from pprint import pprint


def get_objects_handles(sim, excluded_objects=None, verbose=False):
    if excluded_objects is None:
        print("No excluded objects provided")
    objects = {
        sim.getObjectAlias(obj_id): {"id": obj_id}
        # number 2 means get only first level of objects
        for obj_id in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 2)
        if sim.getObjectAlias(obj_id) not in excluded_objects
    }

    if verbose:
        print("Detected objects:")
        for obj, name in objects.items():
            print(f"ID: {obj} Name: {name}")

    print(f"Finished getting object handles. Total objects: {len(objects)}")

    return objects


def get_objects_poses(sim, objects, verbose=False):

    if verbose:
        print("#" * 40)
        print("Objects poses:")
    for name, obj_dict in objects.items():
        id = obj_dict["id"]
        pose = sim.getObjectPose(id)
        objects[name]["pose"] = pose
        if verbose:
            # Print pose with at most 2 decimal places
            formatted_pose = [
                round(x, 2) if isinstance(x, float) else x for x in pose
            ]

            # Alternative approach using a helper function:
            # def format_pose_value(x):
            #     return round(x, 2) if isinstance(x, float) else x
            # formatted_pose = [format_pose_value(x) for x in pose]
            pprint(
                f"ID: {id} | Name: {name} | Pose: {formatted_pose}",
                indent=4,
                width=80,
                compact=True,
            )
            # print(f"ID: {id} | Name: {name} | Pose: {formatted_pose}")
            print("-" * 40)

    print(f"Finished getting object poses. Total objects: {len(objects)}")

    return objects
