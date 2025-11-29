#!/usr/bin/env python3
# coding: utf-8

import math
import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty, EmptyResponse

import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def bresenham(ix0, iy0, ix1, iy1):
    """Cell-by-cell Bresenham between (ix0,iy0) and (ix1,iy1)."""
    dx = abs(ix1 - ix0)
    dy = abs(iy1 - iy0)
    x = ix0
    y = iy0
    sx = 1 if ix0 < ix1 else -1
    sy = 1 if iy0 < iy1 else -1
    if dy <= dx:
        err = dx / 2
        while x != ix1:
            yield x, y
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != iy1:
            yield x, y
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    yield ix1, iy1


class ObstacleMapper(object):
    def __init__(self):
        # Params
        p = rospy.get_param
        self.fixed_frame = p("~fixed_frame", "map")
        self.obstacles_topic = p("~obstacles_topic", "/points/velodyne_obstacles")
        self.grid_topic = p("~grid_topic", "/map/local_map/obstacle")

        self.res = float(p("~resolution", 0.1))
        self.xmin = float(p("~xmin", -20.0))
        self.xmax = float(p("~xmax",  20.0))
        self.ymin = float(p("~ymin", -20.0))
        self.ymax = float(p("~ymax",  20.0))

        self.voxel_leaf = float(p("~voxel_leaf", 0.10))
        self.do_raytrace = bool(p("~do_raytrace", True))
        self.max_range = float(p("~max_range", 0.0))  # 0 = unlimited

        # Log-odds model
        self.l_occ = float(p("~l_occ", 0.85))
        self.l_free = float(p("~l_free", -0.40))
        self.l_min = float(p("~l_min", -2.0))
        self.l_max = float(p("~l_max",  3.5))
        self.p_occ_pub_thresh = float(p("~p_occ_pub_thresh", 0.65))
        self.p_free_pub_thresh = float(p("~p_free_pub_thresh", 0.20))
        self.initialize_unknown = bool(p("~initialize_unknown", True))

        # Optional inflation
        self.inflation_radius = float(p("~inflation_radius", 0.2))
        self.inflation_cells = int(round(self.inflation_radius / self.res)) if self.inflation_radius > 0.0 else 0

        # Derived map size
        self.width = int(round((self.xmax - self.xmin) / self.res))
        self.height = int(round((self.ymax - self.ymin) / self.res))

        # Persistent buffers
        # Use float32 log-odds; and a seen-mask to support UNKNOWN
        self.grid = np.zeros((self.height, self.width), dtype=np.float32)
        self.seen = np.zeros((self.height, self.width), dtype=np.bool_)  # False => unknown

        # TF
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tflist = tf2_ros.TransformListener(self.tfbuf)

        # Pub/Sub
        self.pub = rospy.Publisher(self.grid_topic, OccupancyGrid, queue_size=1, latch=False)
        self.sub = rospy.Subscriber(self.obstacles_topic, PointCloud2, self.cb_cloud, queue_size=10)

        # Services
        rospy.Service("~reset", Empty, self.srv_reset)

        # Timer to publish at steady rate
        self.pub_rate = float(p("~publish_rate_hz", 8.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.pub_rate), self.timer_publish)

        rospy.loginfo("[global_occupancy_grid_mapper] fixed_frame=%s, topic=%s -> %s, map %dx%d @ %.3fm",
                      self.fixed_frame, self.obstacles_topic, self.grid_topic, self.width, self.height, self.res)

    # Simple Empty service without importing std_srvs to keep file compact
    def srv_reset(self, req):
        self.grid.fill(0.0)
        self.seen.fill(False)
        rospy.loginfo("[global_occupancy_grid_mapper] map reset.")
        return EmptyResponse()

    def world_to_cell(self, x, y):
        ix = int((x - self.xmin) / self.res)
        iy = int((y - self.ymin) / self.res)
        return ix, iy

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.width and 0 <= iy < self.height

    def cb_cloud(self, msg):
        # Try to use the transform at the cloud's timestamp first
        stamp = msg.header.stamp
        src_frame = msg.header.frame_id

        try:
            # Wait briefly for the exact transform
            if not self.tfbuf.can_transform(self.fixed_frame, src_frame, stamp, rospy.Duration(0.3)):
                raise tf2_ros.ExtrapolationException("Exact-time TF not available")

            tf = self.tfbuf.lookup_transform(self.fixed_frame, src_frame, stamp, rospy.Duration(0.0))
        except (tf2_ros.ExtrapolationException,
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException) as e:
            # Fall back to the latest available TF
            try:
                tf = self.tfbuf.lookup_transform(self.fixed_frame, src_frame, rospy.Time(0), rospy.Duration(0.2))
                rospy.logwarn_throttle(
                    2.0,
                    "[global_occupancy_grid_mapper] TF %s -> %s extrapolation; using latest transform instead (dt=%.3fs).",
                    src_frame, self.fixed_frame, (rospy.Time.now() - stamp).to_sec()
                )
            except Exception as e2:
                rospy.logwarn_throttle(1.0, "[global_occupancy_grid_mapper] TF %s -> %s failed: %s", src_frame, self.fixed_frame, str(e2))
                return

        # Transform cloud with whichever TF we got
        try:
            cloud_tf = tf2_sm.do_transform_cloud(msg, tf)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "[global_occupancy_grid_mapper] do_transform_cloud failed: %s", str(e))
            return

        # Sensor origin in fixed frame
        sx = tf.transform.translation.x
        sy = tf.transform.translation.y

        # Iterate points (x,y,z); skip NaNs
        pts_iter = pc2.read_points(cloud_tf, field_names=("x", "y", "z"), skip_nans=True)

        # Optional: naive voxel downsample by rounding to leaf
        leaf = self.voxel_leaf
        seen_vox = set() if leaf > 0.0 else None

        max_range_sq = None if self.max_range <= 0.0 else (self.max_range * self.max_range)

        hits = []  # store cell endpoints to also apply inflation in one pass
        for x, y, z in pts_iter:
            if max_range_sq is not None:
                dx = x - sx
                dy = y - sy
                if dx*dx + dy*dy > max_range_sq:
                    continue

            if leaf > 0.0:
                key = (int(math.floor(x / leaf)), int(math.floor(y / leaf)))
                if key in seen_vox:
                    continue
                seen_vox.add(key)

            ix, iy = self.world_to_cell(x, y)
            if not self.in_bounds(ix, iy):
                continue

            # Raytrace free along line from sensor cell to hit cell
            if self.do_raytrace:
                isx, isy = self.world_to_cell(sx, sy)
                for cx, cy in bresenham(isx, isy, ix, iy):
                    if not self.in_bounds(cx, cy):
                        break
                    # do not overwrite the hit itself here; break at the endpoint
                    if cx == ix and cy == iy:
                        break
                    self.grid[cy, cx] = clamp(self.grid[cy, cx] + self.l_free, self.l_min, self.l_max)
                    self.seen[cy, cx] = True

            # Occupied update on hit cell
            self.grid[iy, ix] = clamp(self.grid[iy, ix] + self.l_occ, self.l_min, self.l_max)
            self.seen[iy, ix] = True
            hits.append((ix, iy))

        # Optional inflation (simple disk in grid coords)
        if self.inflation_cells > 0 and hits:
            r = self.inflation_cells
            r2 = r * r
            for ix, iy in hits:
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        if dx*dx + dy*dy > r2:
                            continue
                        cx = ix + dx
                        cy = iy + dy
                        if not self.in_bounds(cx, cy):
                            continue
                        # Slightly lower weight than direct hit so repeated hits still dominate
                        self.grid[cy, cx] = clamp(self.grid[cy, cx] + 0.5 * self.l_occ, self.l_min, self.l_max)
                        self.seen[cy, cx] = True

    def timer_publish(self, _evt):
        # Prepare OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = self.fixed_frame

        meta = MapMetaData()
        meta.resolution = self.res
        meta.width = self.width
        meta.height = self.height
        # origin is the corner of cell (0,0); shift by -res/2 to make cell centers land on xmin/ymin
        meta.origin.position.x = self.xmin - 0.5 * self.res
        meta.origin.position.y = self.ymin - 0.5 * self.res
        grid_msg.info = meta

        data = np.empty((self.height, self.width), dtype=np.int8)

        # Convert log-odds -> probability -> occupancy value
        # p = 1 - 1/(1+exp(l))
        l = self.grid
        p = 1.0 - 1.0 / (1.0 + np.exp(l))
        occ = (p >= self.p_occ_pub_thresh)
        free = (p <= self.p_free_pub_thresh)

        # Unknown handling
        if self.initialize_unknown:
            data[:] = -1
            # Fill known free/occ cells
            data[free & self.seen] = 0
            data[occ & self.seen] = 100
            # mid probabilities remain -1 unless seen is True but neither threshold met; make them "free-ish"
            mid = (~occ) & (~free) & self.seen
            data[mid] = 50
        else:
            data[:] = 0
            data[occ] = 100
            mid = (~occ) & (~free)
            data[mid & self.seen] = 50

        grid_msg.data = data.flatten(order="C").tolist()
        self.pub.publish(grid_msg)


# Tiny shim for std_srvs/Empty without extra import at top
from std_srvs.srv import Empty, EmptyResponse

if __name__ == "__main__":
    rospy.init_node("global_occupancy_grid_mapper")
    ObstacleMapper()
    rospy.spin()
