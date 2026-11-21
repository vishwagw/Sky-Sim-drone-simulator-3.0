## agent_server.gd
## Headless "agent server" — exposes the running sim to an external algorithm
## over WebSocket, one physics tick per action (see docs/agent_protocol.md).
##
## Run:  godot --headless demo/agent_server.tscn -- --port 5557
##
## Attach to a Node in a scene that has a sibling DroneBody. NOTE: this is
## written to-spec; the WebSocket handshake timing and RigidBody teleport are
## the two spots most likely to need a small tweak on first real run.

extends Node

@export var drone: DroneBody
@export var perception: DronePerception    # optional; adds depth/RGB to obs
@export var drone_path: NodePath           # robust wiring for hand-authored scenes
@export var perception_path: NodePath
@export var port: int = 5557
@export var physics_hz: int = 400
@export var camera_hz: int = 30            # sensor rate (<= physics_hz)
@export var bounds: float = 200.0          # out-of-bounds cube half-extent (m)
@export var spawn_default: Vector3 = Vector3(0, 2, 0)
@export var recv_timeout_ms: int = 30000   # give up a blocking recv after this

var _cam_interval: int = 1
var _base_mass: float = 1.0
var _dr = null

var _tcp: TCPServer
var _ws: WebSocketPeer = null
var _hello_sent: bool = false
var _step: int = 0
var _t: float = 0.0

func _ready() -> void:
	if drone == null and not drone_path.is_empty():
		drone = get_node_or_null(drone_path)
	if perception == null and not perception_path.is_empty():
		perception = get_node_or_null(perception_path)
	_parse_args()
	Engine.physics_ticks_per_second = physics_hz
	_cam_interval = max(1, int(round(float(physics_hz) / float(max(1, camera_hz)))))
	# Ground-truth collision reporting.
	if drone:
		drone.contact_monitor = true
		drone.max_contacts_reported = 4
		_base_mass = drone.mass
	_tcp = TCPServer.new()
	var err := _tcp.listen(port)
	if err != OK:
		push_error("agent_server: could not listen on port %d (err %d)" % [port, err])
		return
	print("SkySim agent server listening on ws://127.0.0.1:%d" % port)

func _physics_process(_delta: float) -> void:
	if _ws == null:
		_try_accept()
		return

	_ws.poll()
	var state := _ws.get_ready_state()
	if state == WebSocketPeer.STATE_CONNECTING:
		return                                   # handshake still completing
	if state != WebSocketPeer.STATE_OPEN:
		_disconnect()
		return

	if not _hello_sent:
		_send({
			"type": "hello", "protocol": 1,
			"physics_hz": physics_hz, "dt": 1.0 / float(physics_hz),
			"control_modes": ["attitude", "motors"],
		})
		_hello_sent = true

	# Block until the client sends a command. While blocked the engine is stalled,
	# so physics only advances when we return -> exactly one tick per command.
	var cmd = _recv_blocking()
	if cmd == null:
		_disconnect()
		return

	match str(cmd.get("type", "")):
		"reset":
			_do_reset(cmd)
		"action":
			_apply_action(cmd)
		"close":
			_ws.close()
			_disconnect()
			return
		_:
			_send({"type": "error", "message": "unknown message type"})

	# Reply with the observation of the current state, then return -> integrate
	# one physics tick with the setpoint/reset we just applied.
	_send_obs()

# --------------------------------------------------------------------------
# Connection handling
# --------------------------------------------------------------------------
func _try_accept() -> void:
	if _tcp.is_connection_available():
		var conn := _tcp.take_connection()
		_ws = WebSocketPeer.new()
		_ws.accept_stream(conn)
		_hello_sent = false
		print("agent_server: client connected")

func _disconnect() -> void:
	_ws = null
	_hello_sent = false
	print("agent_server: client disconnected")

func _recv_blocking():
	var waited := 0
	while true:
		_ws.poll()
		if _ws.get_ready_state() != WebSocketPeer.STATE_OPEN:
			return null
		while _ws.get_available_packet_count() > 0:
			var txt := _ws.get_packet().get_string_from_utf8()
			var parsed = JSON.parse_string(txt)
			if parsed is Dictionary:
				return parsed
			# ignore malformed frames
		OS.delay_msec(1)
		waited += 1
		if waited > recv_timeout_ms:
			return null

# --------------------------------------------------------------------------
# Sim control
# --------------------------------------------------------------------------
func _do_reset(cmd: Dictionary) -> void:
	var spawn := spawn_default

	if cmd.get("spawn", null) != null:
		var s = cmd["spawn"]
		if s is Array and s.size() == 3:
			spawn = Vector3(s[0], s[1], s[2])

	# --- Domain randomization: applied where the engine allows, always echoed
	#     so the client can log exactly what this episode used. ---
	_dr = null
	var basis := Basis.IDENTITY
	if cmd.get("randomize", null) is Dictionary:
		var dr: Dictionary = cmd["randomize"]
		if dr.has("mass_scale"):
			drone.mass = _base_mass * float(dr["mass_scale"])
		if dr.get("spawn_jitter", null) is Array and dr["spawn_jitter"].size() == 3:
			var j = dr["spawn_jitter"]
			spawn += Vector3(j[0], j[1], j[2])
		if dr.get("attitude_jitter_deg", null) is Array and dr["attitude_jitter_deg"].size() == 3:
			var a = dr["attitude_jitter_deg"]
			basis = Basis.from_euler(Vector3(deg_to_rad(a[0]), deg_to_rad(a[1]), deg_to_rad(a[2])))
		# wind and sensor noise go through DroneBody setters; has_method() keeps older extension
		#     binaries working. sensor_noise is a multiplier on default noise (0 = none, 1 = default).
		if dr.has("wind") and drone.has_method("set_wind"):
			var w = dr["wind"]
			drone.call("set_wind", Vector3(w[0], w[1], w[2]))
		if dr.has("sensor_noise") and drone.has_method("set_sensor_noise_scale"):
			drone.call("set_sensor_noise_scale", float(dr["sensor_noise"]))
		_dr = dr

	# Teleport the RigidBody and clear its motion.
	drone.global_transform = Transform3D(basis, spawn)
	drone.linear_velocity = Vector3.ZERO
	drone.angular_velocity = Vector3.ZERO

	drone.disarm()
	if bool(cmd.get("arm", true)):
		drone.arm()
	# Neutral setpoint so the settling tick after reset is benign.
	drone.set_attitude_setpoint(0.0, 0.0, 0.0, 0.0)

	_step = 0
	_t = 0.0

func _apply_action(cmd: Dictionary) -> void:
	var mode := str(cmd.get("mode", "attitude"))
	if mode == "motors":
		var arr = cmd.get("throttles", [])
		var pf := PackedFloat64Array()
		for v in arr:
			pf.append(float(v))
		drone.set_rotor_throttles(pf)
	else:
		drone.set_attitude_setpoint(
			float(cmd.get("roll", 0.0)),
			float(cmd.get("pitch", 0.0)),
			float(cmd.get("yaw_rate", 0.0)),
			float(cmd.get("throttle", 0.0)))

func _send_obs() -> void:
	_t += 1.0 / float(physics_hz)
	_step += 1
	var b := drone.global_transform.basis
	var q := b.get_rotation_quaternion()
	var e := b.get_euler()          # radians (YXZ)
	var p := drone.global_position
	var oob := (absf(p.x) > bounds or absf(p.z) > bounds or p.y > bounds or p.y < -1.0)
	var colliding := false
	if drone.has_method("get_colliding_bodies"):
		colliding = drone.get_colliding_bodies().size() > 0

	# Sensors run at camera_hz (a fraction of physics). On non-sensor ticks these
	# stay null and the client holds the previous frame.
	var depth = null
	var camera = null
	if perception and (_step % _cam_interval == 0):
		depth = perception.sample_depth()
		if perception.enable_rgb:
			var rgb = perception.sample_rgb()
			if not rgb.is_empty():
				camera = rgb

	_send({
		"type": "obs",
		"t": _t, "step": _step,
		"gt": {
			"pos": [p.x, p.y, p.z],
			"vel": [drone.linear_velocity.x, drone.linear_velocity.y, drone.linear_velocity.z],
			"quat": [q.x, q.y, q.z, q.w],
			"euler": [e.x, e.y, e.z],
			"ang_vel": [drone.angular_velocity.x, drone.angular_velocity.y, drone.angular_velocity.z],
		},
		"telemetry": drone.get_telemetry(),
		"camera": camera,
		"depth": depth,
		"collision": colliding,
		"out_of_bounds": oob,
		"dr": _dr,
	})

func _send(obj: Dictionary) -> void:
	if _ws and _ws.get_ready_state() == WebSocketPeer.STATE_OPEN:
		_ws.send_text(JSON.stringify(obj))

func _parse_args() -> void:
	var args := OS.get_cmdline_user_args()   # args after "--"
	for i in args.size():
		if args[i] == "--port" and i + 1 < args.size():
			port = int(args[i + 1])
