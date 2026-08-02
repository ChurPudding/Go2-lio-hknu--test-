p='tools/lio_health.py'
s=open(p).read()
if "out_topic" in s:
    print('이미 적용됨'); raise SystemExit
s=s.replace("self.declare_parameter('yaw_rate_max'",
"""self.declare_parameter('out_topic', '/indoor/health')
        self.declare_parameter('out_info_topic', '/indoor/health_info')
        self.declare_parameter('yaw_rate_max'""")
s=s.replace("self.pub = self.create_publisher(Bool, '/lio/health', 10)",
            "self.pub = self.create_publisher(Bool, g('out_topic'), 10)")
s=s.replace("self.pub_info = self.create_publisher(String, '/lio/health_info', 10)",
            "self.pub_info = self.create_publisher(String, g('out_info_topic'), 10)")
s=s.replace("self.create_subscription(Empty, '/lio/health_reset', self.on_reset, 10)",
            "self.create_subscription(Empty, g('out_topic') + '_reset', self.on_reset, 10)")
open(p,'w').write(s)
print('ok')
