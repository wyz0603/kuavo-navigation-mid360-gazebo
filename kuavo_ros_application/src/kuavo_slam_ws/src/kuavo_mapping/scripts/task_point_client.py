#!/usr/bin/env python3
import rospy
import argparse
from kuavo_mapping.srv import TaskPointOperation, TaskPointOperationRequest, TaskPointOperationResponse
from kuavo_mapping.msg import TaskPoint

def parse_args():
    parser = argparse.ArgumentParser(description="Task Point Command Line Tool")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # add
    parser_add = subparsers.add_parser("add")
    parser_add.add_argument("--name", default="")
    parser_add.add_argument("--use_robot_current_pose", type=bool, default=False)

    # update
    parser_update = subparsers.add_parser("update")
    parser_update.add_argument("--name", required=True)
    parser_update.add_argument("--use_robot_current_pose", type=bool, default=False)

    # delete
    parser_delete = subparsers.add_parser("delete")
    parser_delete.add_argument("--name", required=True)

    # get
    parser_get = subparsers.add_parser("get")  # no arguments

    return parser.parse_args()

def print_task_points(task_points):
    if not task_points:
        print("📭 No task points found.")
        return
    print("📌 Task Points:")
    for i, tp in enumerate(task_points):
        print(f"  {i+1}. name: {tp.name}, x: {tp.pose.position.x:.2f}, y: {tp.pose.position.y:.2f}, z: {tp.pose.position.z:.2f}")

def main():
    args = parse_args()
    rospy.wait_for_service("/task_point")
    client = rospy.ServiceProxy("/task_point", TaskPointOperation)

    req = TaskPointOperationRequest()
    # res = TaskPointOperationResponse()
    # print(req)
    # print(res)
    # exit()
    if args.command == "add":
        req.operation = 0
        req.name = args.name
        req.use_robot_current_pose = args.use_robot_current_pose

    elif args.command == "update":
        req.operation = 1
        req.name = args.name
        req.use_robot_current_pose = args.use_robot_current_pose

    elif args.command == "delete":
        req.operation = 2
        req.name = args.name

    elif args.command == "get":
        req.operation = 3

    try:
        resp = client.call(req)
        if resp.success:
            print("✅ Success:", resp.message)
            if args.command == "get":
                print_task_points(resp.task_points)
        else:
            print("❌ Failed:", resp.message)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == '__main__':
    main()