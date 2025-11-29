#!/usr/bin/env python3
import os, time, subprocess, shlex
import rospy

def wait_file_stable(path, idle_s=5):
    rospy.loginfo("Waiting for bag to appear: %s", path)
    while not os.path.exists(path):
        rospy.sleep(0.5)
    rospy.loginfo("Bag found. Waiting for file to become stable...")
    last = -1
    still = 0
    while still < idle_s:
        sz = os.path.getsize(path)
        if sz == last:
            still += 1
        else:
            still = 0
            last = sz
        rospy.sleep(1.0)
    rospy.loginfo("Bag looks stable (size=%d).", last)

def run(cmd_list):
    cmd = " ".join(cmd_list)
    rospy.loginfo("RUN: %s", cmd)
    # Use shell so conda PATH etc. works; ensure you launch from an env where evo is installed
    subprocess.run(cmd, shell=True, check=True)

def main():
    rospy.init_node("run_evo_metrics")
    bag       = rospy.get_param("~bag")
    gt_topic  = rospy.get_param("~gt_topic", "/ground_truth/pose")
    est_topic = rospy.get_param("~est_topic", "/orb_slam3/odom")
    t_offset  = float(rospy.get_param("~t_offset", 0.0))
    n_to_align= int(rospy.get_param("~n_to_align", 50))
    t_max_diff= float(rospy.get_param("~t_max_diff", 0.5))
    out_dir   = rospy.get_param("~out_dir", os.path.expanduser("~/evo_out"))

    os.makedirs(out_dir, exist_ok=True)
    wait_file_stable(bag)

    base = f"--ref {} --t_offset {t_offset} --align --n_to_align {n_to_align} --t_max_diff {t_max_diff}"

    # Trajectory overlay (XY) – save figure
    run([
        "evo_traj bag",
        shlex.quote(bag),
        gt_topic, est_topic,
        base,
        "--plot_mode xy",
        f"--save_plot {shlex.quote(os.path.join(out_dir,'traj_xy.png'))}"
    ])

    # APE (absolute pose error) – saves plot, JSON, CSV table
    run([
        "evo_ape bag",
        shlex.quote(bag),
        gt_topic, est_topic,
        base,
        f"--save_results {shlex.quote(os.path.join(out_dir,'ape.json'))}",
        f"--save_plot {shlex.quote(os.path.join(out_dir,'ape.png'))}",
        f"--save_table {shlex.quote(os.path.join(out_dir,'ape_table.csv'))}"
    ])

    # RPE translation per 1s
    run([
        "evo_rpe bag",
        shlex.quote(bag),
        gt_topic, est_topic,
        base, "-r trans_part", "--delta 1", "--delta_unit s",
        f"--save_results {shlex.quote(os.path.join(out_dir,'rpe_trans_1s.json'))}",
        f"--save_plot {shlex.quote(os.path.join(out_dir,'rpe_trans_1s.png'))}",
        f"--save_table {shlex.quote(os.path.join(out_dir,'rpe_trans_1s.csv'))}"
    ])

    # RPE rotation per 1s
    run([
        "evo_rpe bag",
        shlex.quote(bag),
        gt_topic, est_topic,
        base, "-r rot_part", "--delta 1", "--delta_unit s",
        f"--save_results {shlex.quote(os.path.join(out_dir,'rpe_rot_1s.json'))}",
        f"--save_plot {shlex.quote(os.path.join(out_dir,'rpe_rot_1s.png'))}",
        f"--save_table {shlex.quote(os.path.join(out_dir,'rpe_rot_1s.csv'))}"
    ])

    rospy.loginfo("evo metrics done. See %s", out_dir)

if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        rospy.logerr("evo command failed: %s", e)
    except Exception as e:
        rospy.logerr("unexpected error: %s", e)
