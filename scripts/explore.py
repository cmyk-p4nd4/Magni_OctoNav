#!/usr/bin/env python3

from __future__ import annotations

import rospy
from rospy.timer import TimerEvent
import cv2
import numpy as np
from threading import Thread, Lock, Event, Condition
from queue import Queue, Empty, Full
from matplotlib import pyplot as plt
from matplotlib.image import AxesImage
import time
from colorama import Fore, Style
import gc


from std_msgs.msg import ColorRGBA, Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Vector3, Quaternion
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseAction, MoveBaseGoal
import actionlib
import tf
from visualization_msgs.msg import Marker, MarkerArray

class FrontierExplore(object):
	def __init__(self):
		self.gridMap: OccupancyGrid = OccupancyGrid()
		self.message_lock = Lock()

		self.debug_plot = rospy.get_param("~debug_plot", False)
		self.show_rviz = rospy.get_param("~visualize", True)
		self.map_frame = rospy.get_param("~map_frame", "map")
		self.base_frame = rospy.get_param("~base_frame", "base_link")

		self.cancel_goal_service = rospy.Service(rospy.get_name()+"/cancel_goal", Empty, self.cancel_goal)

		self.tl = tf.TransformListener()
		self.global_map_sub = rospy.Subscriber(rospy.get_param("~map_topic", "/projected_map"), OccupancyGrid, self.occ_msg_handler, queue_size=1)
		
		self.action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		self.goal_done_evt = Event()

		self.backlisted_goal: list[Point] = []
		self.backlisted_zone_radius = 0.2

		if self.show_rviz:
			self.frontier_marker_pub = rospy.Publisher(rospy.get_name()+"/markers", MarkerArray, queue_size=1)

		self.process_time_pub = rospy.Publisher(rospy.get_name()+"/time", Float64, queue_size=1, latch=True)

		self.map_last_origin = np.zeros((2,), dtype=np.float64)
		self.map_resolution = 0.05

		self.visualize_thread = Thread(target=self.visualize_image_thread)
		self.stop_vis_evt = Event()
		self.vis_data_queue = Queue()
		self.vis_cv = Condition()

		self.plotting_thread = Thread(target=self.show_plot)


	def __del__(self):
			self.stop()

	def run(self):
		rospy.Timer(rospy.Duration(1), callback=self.process, oneshot=True)

	def stop(self):
			self.global_map_sub.unregister()
			if self.visualize_thread.is_alive():
				self.stop_vis_evt.set()
				self.visualize_thread.join()

	def occ_msg_handler(self, map: OccupancyGrid) -> None:
		with self.message_lock:
			self.grid_data = np.array(map.data, dtype=np.int16).reshape((map.info.height, map.info.width))
			self.map_resolution = map.info.resolution
			point: Point = map.info.origin.position
			self.map_last_origin = np.array([point.x, point.y], dtype=np.float64)
			
	
	def process(self, event: TimerEvent | None = None):
		grid_data = None
		if not hasattr(self, "grid_data"):
			rospy.Timer(rospy.Duration(1), callback=self.process, oneshot=True)
			return

		self.message_lock.acquire(blocking=True)
		grid_data = self.grid_data.copy()
		last_origin = self.map_last_origin
		map_resolution = self.map_resolution
		self.message_lock.release()
		
		start = time.thread_time()

		# results are in (M, 2) shape
		found_goals_cell = self.find_frontier(grid_data.copy())

		if len(found_goals_cell) < 1:
			rospy.loginfo(Fore.GREEN+Style.BRIGHT+"Mapping Completed"+Style.RESET_ALL)
			return

		if not self.tl.canTransform(self.map_frame, self.base_frame, rospy.Time.now() - rospy.Duration.from_sec(0.5)):
			return

		x, y = 0, 0
		try:
			(x, y, _), (__) = self.tl.lookupTransform(self.map_frame, self.base_frame, rospy.Time())
		except (tf.LookupException, tf.ConnectivityException):
			rospy.logwarn_throttle(
				1, f"Unable to lookup transform from {self.map_frame} --> {self.base_frame} at {rospy.Time.now().to_sec()}")

		# # shape in (1,2) --> (2, )
		base_pose = np.array([x, y])
		found_goals = found_goals_cell * map_resolution + last_origin
		dist = np.linalg.norm(found_goals - base_pose, axis=-1)

		s_idx = np.argsort(dist, kind='heapsort')

		p = s_idx[0] 
		for idx in s_idx:
			goal_is_valid = True
			p = idx
			
			# we check if this potential goal is valid from a list of unreachable goal
			# by computing the distance between all blacklisted goals
			for bl_point in self.backlisted_goal:
				goal2bl_dist = np.linalg.norm(found_goals[idx] - np.array([bl_point.x, bl_point.y]), axis=-1)
				
				if goal2bl_dist < self.backlisted_zone_radius:
					goal_is_valid = False
					break

			if goal_is_valid:
				break
			if goal_is_valid == False and p == s_idx[-1]:
				rospy.loginfo(Fore.GREEN+Style.BRIGHT+"Mapping Completed"+Style.RESET_ALL)
				return

		nearest = Point(x=found_goals[p, 0],
										y=found_goals[p, 1], z=0.0)
		
		end = time.thread_time()
		t_diff = end - start
		self.process_time_pub.publish(t_diff)

		self.current_goal = nearest
		
		# send this goal to action server
		self.send_goal(nearest)
	
	def send_goal(self, point: Point):
		goal = MoveBaseGoal()
		pose = goal.target_pose
		pose.header.frame_id = self.map_frame
		pose.header.stamp = rospy.Time.now()

		pose.pose.position = point
		pose.pose.orientation = Quaternion(w=1.0)
		goal.target_pose = pose
		
		self.action_client.send_goal(goal, done_cb=self.action_done_handler,
		                             active_cb=self.active_state_handler, 
																 feedback_cb=self.action_feedback_handler)


	def cancel_goal(self, req):
		self.action_client.cancel_goal()
		return []

	def action_feedback_handler(self, feedback: MoveBaseActionFeedback):

		pass

	def active_state_handler(self):
		pass

	def action_done_handler(self, state: actionlib.GoalStatus, result = None):
		
		if state == actionlib.GoalStatus.ABORTED:
			# failed, blacklist this goal
			self.backlisted_goal.append(self.current_goal)
			f_text = Style.BRIGHT + Fore.LIGHTCYAN_EX + \
				f"Cannot reach Goal ({self.current_goal.x:.3f},{self.current_goal.y:.3f}). Blacklisting" + Style.RESET_ALL
			rospy.loginfo(f_text)

		rospy.Timer(rospy.Duration.from_sec(0.5), self.process, oneshot=True, reset=True)


	def show_frontier_markers(self, frontiers: list[np.ndarray]):
		"""
			`frontiers` is a list of found vertices that are open-ended for exploration, \n
			Each of frontiers consist of cell index (x, y)
			"""
		marker_list = MarkerArray()
		stamp = rospy.Time.now()

		# first flush all previous markers
		preflush_marker = Marker()
		preflush_marker.header.frame_id = 'map'
		preflush_marker.header.stamp = stamp
		preflush_marker.action = Marker.DELETEALL
		preflush_marker.id = -1
		marker_list.markers.append(preflush_marker)

		with self.message_lock:
			last_origin = self.map_last_origin
			map_resolution = self.map_resolution

		m_id = 0

		for frontier in frontiers:
			if frontier.shape[0] %2:
				frontier = np.pad(frontier, ((0,1), (0,0)), 'wrap')
			v1, v2 = np.vsplit(frontier, 2)
			# array in CCW ordering
			# the second array needs to be reverse indexed
			w_frontier = v2[::-1]

			# set the first vertex with sphere
			first = (w_frontier[0]) * map_resolution + last_origin
			vtx_start = Marker()
			vtx_start.header.frame_id = self.map_frame
			vtx_start.header.stamp = stamp
			vtx_start.ns = "vertex_start"
			vtx_start.id = m_id
			vtx_start.type = Marker.SPHERE
			vtx_start.lifetime = rospy.Duration(10)
			vtx_start.frame_locked = True

			vtx_start.pose.orientation.w = 1.0
			vtx_start.pose.position.x = first[0]
			vtx_start.pose.position.y = first[1]
			vtx_start.pose.position.z = 0
			vtx_start.scale = Vector3(x=0.3, y=0.3, z=0.3)
			vtx_start.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
			marker_list.markers.append(vtx_start)

			m_id += 1

			vtx_arc = Marker()
			vtx_arc.header.frame_id = self.map_frame
			vtx_arc.header.stamp = stamp
			vtx_arc.ns = "vertex_trace"
			vtx_arc.id = m_id
			vtx_arc.type = Marker.LINE_STRIP
			vtx_arc.lifetime = rospy.Duration(10)
			vtx_arc.frame_locked = True

			vtx_arc.pose.orientation.w = 1.0
			vtx_arc.pose.position.x = first[0]
			vtx_arc.pose.position.y = first[1]
			vtx_arc.pose.position.z = 0
			vtx_arc.scale = Vector3(x=0.1)

			w_first = w_frontier[0]
			# the rest of the points are have relative offset to the first point
			v = np.subtract(w_frontier, w_first) * map_resolution

			w_points = []
			for p in v:
				w_point = Point(x=p[0], y=p[1], z=0.0)
				w_points.append(w_point)

			vtx_arc.points = w_points
			vtx_arc.colors = [ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)] * len(w_points)
			marker_list.markers.append(vtx_arc)

			m_id += 1

		if hasattr(self, "frontier_marker_pub"):
			if self.frontier_marker_pub.get_num_connections() > 0:
				self.frontier_marker_pub.publish(marker_list)

	def update(self):
		if self.vis_data_queue.empty():
			return
		try:
			data = self.vis_data_queue.get_nowait()
		except Empty:
			return None
		return data

	def visualize_image_thread(self):
		fig = plt.figure()
		fig.set_dpi(300)
		plt.ion()
		axs = fig.subplots(1,2)
		im: list[AxesImage] = []
		for ax in axs:
			im.append(ax.imshow(np.zeros((30, 30), dtype=np.uint8), cmap='gist_gray', vmin=0, vmax=255, aspect='auto'))

		while not self.stop_vis_evt:
			with self.vis_cv:
				self.vis_cv.wait_for(self.vis_data_queue.full)
				d1, d2 = self.vis_data_queue.get(block=False)

			im[0].set_data(d1)
			im[1].set_data(d2)

			fig.canvas.draw_idle()
			fig.canvas.flush_events()

		plt.ioff()
		plt.close(fig)

	def show_plot(self, grid: np.ndarray, frame: np.ndarray, bbox: list):
		dst = frame.copy().astype(np.uint8)
		dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
		for box in bbox:
			cv2.drawContours(dst, box, 0, (0, 255, 0), 2)
		with self.vis_cv:
			try:
				self.vis_data_queue.put_nowait((np.array(grid, dtype=np.uint8), np.array(dst, dtype=np.uint8)))
			except Full:
				pass
			self.vis_cv.notify()

	def image_gradient(self, mat, ksize=3) -> np.ndarray:
		gX = cv2.Sobel(mat, ddepth=cv2.CV_16SC1, dx=1, dy=0, ksize=ksize)
		gY = cv2.Sobel(mat, ddepth=cv2.CV_16SC1, dx=0, dy=1, ksize=ksize)
		gX = cv2.convertScaleAbs(gX)
		gY = cv2.convertScaleAbs(gY)
		return cv2.addWeighted(gX, 0.5, gY, 0.5, 0).astype(np.int16)

	def find_frontier(self, grid):
		pad_size = 50
		grid = cv2.copyMakeBorder(grid, pad_size, pad_size, pad_size, pad_size, cv2.BORDER_CONSTANT, value=-1)
		grid = cv2.GaussianBlur(grid, (3, 3), 0)
		grid[grid > 0] = 200
		grid[grid == 0] = 25
		grid[grid == -1] = 0

		# grid = cv2.dilate(grid, kernel)
		grid = np.array(grid, dtype=np.int16)
		frame = self.image_gradient(grid)
		frame = np.where(((frame % 25) == 0) & (frame <= 100), frame, 0)
		frame[frame > 0] = 255

		points: list[list[int]] = []
		contours, _ = cv2.findContours(frame.copy().astype(
													np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		bboxs = []
		trace_vtx_list = []
		for c in contours:
			rect = cv2.minAreaRect(c)
			if (rect[1][0] * rect[1][1] * self.map_resolution ** 2.0 < 0.25):
					continue
			middle = [int(rect[0][0]-50), int(rect[0][1]-50)]
			points.append(middle)

			box = cv2.boxPoints(rect)
			box = np.intp(box)
			bboxs.append([box])
			trace_vtx = np.array(c).reshape((-1, 2)).astype(np.int32) - np.array([50, 50])
			trace_vtx_list.append(trace_vtx)

		if self.debug_plot:
			self.show_plot(grid, frame, bboxs)

		if self.show_rviz:
			self.show_frontier_markers(trace_vtx_list)

		return np.array(points)


def main():
	rospy.init_node("frontier_explore")
	rate = rospy.Rate(50.0)
	explore = FrontierExplore()
	explore.run()
	gc.enable()
	while not rospy.is_shutdown():
		try:
			rate.sleep()
		except rospy.ROSInterruptException:
			pass

	explore.stop()
	print()

if __name__ == "__main__":
	main()