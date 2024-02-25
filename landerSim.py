import tkinter as tk
import math
from gptJoy import Joystick
import sys
import webbrowser

class landerSim:
    def __init__(self, root, width, height):
        self.root = root
        self.width = width
        self.height = height

        self.joystick = Joystick(200, self.root)
        self.joystick.canvas.grid(row=0, column=4, rowspan=1, columnspan=1)
        self.use_joystick = True

        self.draw_cg_forces = True

        self.dt = 0.02 # seconds, 50Hz

        # create blank canvas
        self.canvas = tk.Canvas(self.root, width=width, height=height, bg='black')
        self.canvas.grid(row=0, column=0, rowspan=2, columnspan=4)

        # user input default values on launch
        self.usrCmd_gravity_ms2 = 1.62
        self.usrCmd_thrust_norm = 50
        self.usrCmd_thrust_angle_deg = 0.0
        # lander physical properties
        self.mass_kg = 1908 # odysseus 
        self.height_m = 4.0 # odysseus 
        self.width_m = 1.57 # odysseus 
        self.leg_width_m = 4.6 # odysseus 
        # user init conditions
        self.usr_alt_m_init = 8.0
        self.usr_vx_ms_init = 1.0
        self.usr_vz_ms_init = -0.5
        self.usr_phi_deg_init = 0.0

        # User input sliders
        self.thrust_slider = tk.Scale(self.root, from_=100, to=0, resolution=0.01, orient=tk.VERTICAL, label="Thrust", length = 200, command=self.update_thrust_norm_cmd)
        self.thrust_slider.grid(row=0, column=5, padx=4)
        self.thrust_slider.set(self.usrCmd_thrust_norm)
        # add a label to specify thrust range
        self.thrust_label_1 = tk.Label(self.root, text="(2G Local)")
        self.thrust_label_1.place(x=1230, y=45)
        self.thrust_label_2 = tk.Label(self.root, text="(0G Local)")
        self.thrust_label_2.place(x=1230, y=255)

        self.angle_slider = tk.Scale(self.root, from_=10, to=-10, resolution=0.01, orient=tk.HORIZONTAL, label="Thrust Angle", length = 100, command=self.update_thrust_angle_deg_cmd)
        self.angle_slider.grid(row=1, column=4, pady=4)
        self.angle_slider.set(self.usrCmd_thrust_angle_deg)

        self.gravity_slider = tk.Scale(self.root, from_=20, to=0.05, resolution=0.01, orient=tk.VERTICAL, length = 400, label="Gravity (m/s^2)", command=self.update_gravity_ms2)
        self.gravity_slider.grid(row=0, column=6, rowspan=3, padx=4) 
        self.gravity_slider.set(self.usrCmd_gravity_ms2)
        # add a label to specify different planets
        self.gravity_label_1 = tk.Label(self.root, text="Moon")
        self.gravity_label_1.place(x=1485, y=418)
        self.gravity_label_2 = tk.Label(self.root, text="Mars")
        self.gravity_label_2.place(x=1485, y=380)
        self.gravity_label_3 = tk.Label(self.root, text="Earth")
        self.gravity_label_3.place(x=1485, y=268)

        self.mass_slider = tk.Scale(self.root, from_=5000, to=500, resolution=0.01, orient=tk.VERTICAL, length = 400, label="Lander Mass (kg)", command=self.update_mass_kg)
        self.mass_slider.grid(row=5, column=0, rowspan=3, padx=4)
        self.mass_slider.set(self.mass_kg)
        # add a label to call out oddysseus mass
        self.mass_label_1 = tk.Label(self.root, text="Odysseus Lander")
        self.mass_label_1.place(x=105, y=810)

        self.height_slider = tk.Scale(self.root, from_=20, to=2, resolution=0.01, orient=tk.VERTICAL, length = 400, label="Lander Height (m)", command=self.update_height_m)
        self.height_slider.grid(row=5, column=1, rowspan=3, padx=4)
        self.height_slider.set(self.height_m)
        # add a label to call out oddysseus height
        self.hieght_label_1 = tk.Label(self.root, text="Odysseus Lander")
        self.hieght_label_1.place(x=350, y=885)

        self.width_slider = tk.Scale(self.root, from_=4, to=1, resolution=0.01, orient=tk.VERTICAL, length = 400, label="Lander Width (m)", command=self.update_width_m)
        self.width_slider.grid(row=5, column=2, rowspan=3, padx=4)
        self.width_slider.set(self.width_m)
        # add a label to call out oddysseus width
        self.width_label_1 = tk.Label(self.root, text="Odysseus Lander")
        self.width_label_1.place(x=592, y=855)

        self.lg_width_slider = tk.Scale(self.root, from_=9, to=1.5, resolution=0.01, orient=tk.VERTICAL, length = 400, label="Lander Leg Width (m)", command=self.update_lg_width_m)
        self.lg_width_slider.grid(row=5, column=3, rowspan=3, padx=4)
        self.lg_width_slider.set(self.leg_width_m)
        # add a label to call out oddysseus width
        self.lg_width_label_1 = tk.Label(self.root, text="Odysseus Lander")
        self.lg_width_label_1.place(x=830, y=775)

        self.init_alt_slider = tk.Scale(self.root, from_=50, to=5, resolution=0.01, orient=tk.VERTICAL, length = 400, label="Initial Altitude (m)", command=self.update_init_alt)
        self.init_alt_slider.grid(row=5, column=4, rowspan=3, padx=4)
        self.init_alt_slider.set(self.usr_alt_m_init - self.height_m/2)

        self.init_Vx_slider = tk.Scale(self.root, from_=20, to=0, resolution=0.01, orient=tk.VERTICAL, length = 400, label="Initial Forward Vel (m/s)", command=self.update_init_Vx)
        self.init_Vx_slider.grid(row=5, column=5, rowspan=3, padx=4)
        self.init_Vx_slider.set(self.usr_vx_ms_init)

        self.init_Vz_slider = tk.Scale(self.root, from_=10, to=-10, resolution=0.01, orient=tk.VERTICAL, length = 400, label="Initial Vertical Vel (m/s)", command=self.update_init_Vz)
        self.init_Vz_slider.grid(row=5, column=6, rowspan=3, padx=4)
        self.init_Vz_slider.set(self.usr_vz_ms_init)

        self.init_phi_slider = tk.Scale(self.root, from_=-90, to=90, resolution=0.01, orient=tk.VERTICAL, length = 400, label="Initial Attitude (deg)", command=self.update_init_phi)
        self.init_phi_slider.grid(row=5, column=7, rowspan=3, padx=4)
        self.init_phi_slider.set(self.usr_phi_deg_init)


        self.reset_button = tk.Button(self.root, text="RESET SIM", command=self.initSim)
        self.reset_button.grid(row=2, column=3, pady=10)

        self.use_joystick_button = tk.Button(self.root, text="Toggle Joystick/Slider Control", command=self.toggle_joystick)
        self.use_joystick_button.grid(row=2, column=4, pady=10)

        self.toggle_forces_button = tk.Button(self.root, text="Toggle CG/Force Viz", command=self.toggle_force_viz)
        self.toggle_forces_button.grid(row=2, column=5, pady=10)

        self.zoom_plus_button = tk.Button(self.root, text="Zoom In", command=self.zoom_plus)
        self.zoom_plus_button.grid(row=2, column=1, pady=10)
        self.zoom_minus_button = tk.Button(self.root, text="Zoom Out", command=self.zoom_minus)
        self.zoom_minus_button.grid(row=2, column=2, pady=10)
        self.scale_factor = 30.0 # m to pixel scale factor for animation
        self.sf_max = 70
        self.sf_min = 3.0

        self.subscribe_button = tk.Button(self.root, text="Subscribe to Nicholas Rehm on YouTube!", command=self.subscribe)
        self.subscribe_button.grid(row=2, column=7, pady=10)

        self.subscribe_button = tk.Button(self.root, text="Follow Nicholas Rehm on Twitter!", command=self.follow)
        self.subscribe_button.grid(row=2, column=6, pady=10)

        # set sim initial conditions
        self.initSim()

        # Start it up
        self.update()
    
    # Callbacks
    def update_thrust_norm_cmd(self, val):
        self.usrCmd_thrust_norm = float(val)

    def update_thrust_angle_deg_cmd(self, val):
        self.usrCmd_thrust_angle_deg = float(val)

    def update_gravity_ms2(self, val):
        self.usrCmd_gravity_ms2 = float(val)

    def subscribe(self):
        webbrowser.open_new("https://www.youtube.com/nicholasrehm")

    def follow(self):
        webbrowser.open_new("https://twitter.com/Nicholas_Rehm")

    def toggle_force_viz(self):
        self.draw_cg_forces = not self.draw_cg_forces

    def update_mass_kg(self, val):
        self.mass_kg = float(val)

    def update_height_m(self, val):
        self.height_m = float(val)

    def update_width_m(self, val):
        self.width_m = float(val)
        if self.width_m > self.leg_width_m:
            self.leg_width_m = self.width_m*1.01

    def update_lg_width_m(self, val):
        if float(val) > self.width_m:
            self.leg_width_m = float(val)

    def update_init_alt(self, val):
        self.usr_alt_m_init = float(val) + self.height_m/2

    def update_init_Vx(self, val):
        self.usr_vx_ms_init = float(val)

    def update_init_Vz(self, val):
        self.usr_vz_ms_init = float(val)

    def update_init_phi(self, val):
        self.usr_phi_deg_init = float(val)

    def zoom_plus(self):
        self.scale_factor += 5.0
        self.scale_factor = max(min(self.scale_factor, self.sf_max), self.sf_min)
        
    def zoom_minus(self):
        self.scale_factor -= 5.0
        self.scale_factor = max(min(self.scale_factor, self.sf_max), self.sf_min)

    def toggle_joystick(self):
        self.use_joystick = not self.use_joystick

    # Stuffs
    def initSim(self):
        # state machine shit
        self.touchdown = False
        self.converted_momentum_2_rotation = False
        self.left_leg_first_touch = False
        self.right_leg_first_touch = False
        self.crashed = False

        # lander initial conditions - 3DOF sim
        self.Pz_m = self.usr_alt_m_init # height above ground - up is positive
        self.Px_m = -10.0 # side-side position - to the right is positive
        self.Ax_ms2 = 0.0 # positive to the right
        self.Az_ms2 = 0.0 # positive up
        self.Vx_ms = self.usr_vx_ms_init # positive to the right
        self.Vz_ms = self.usr_vz_ms_init # positive up
        self.phi_deg = self.usr_phi_deg_init # positive rolling to right
        self.p_dps = 0.0 # positive rolling to right
        self.pdot_dps = 0.0 # positive rolling to right

    def update(self):
        self.inertia_kgm2 = 1/4 * self.mass_kg * (self.width_m/2)**2 + 1/12 * self.mass_kg * self.height_m**2 # approximate as cylinder
        self.landed_inertia_kgm2 = 1/4 * self.mass_kg * (self.width_m/2)**2 + 1/3 * self.mass_kg * self.height_m**2 # approximate as cylinder on its end

        self.touchdown_check() # check if we've touched the ground

        # no legs have touched the ground yet, so fly basic 3dof dynamics
        if not self.touchdown:
            # update cmds via joystick
            if self.use_joystick:
                self.thrust_slider.config(state="normal")
                self.angle_slider.config(state="normal")
                self.usrCmd_thrust_angle_deg = -self.joystick.x * 10.0
                self.usrCmd_thrust_norm = (self.joystick.y + 1.0)/2 * 100.0
                self.thrust_slider.set(self.usrCmd_thrust_norm)
                self.angle_slider.set(self.usrCmd_thrust_angle_deg)
                self.thrust_slider.config(state="disabled")
                self.angle_slider.config(state="disabled")
            else:
                self.thrust_slider.config(state="normal")
                self.angle_slider.config(state="normal")

            self.engine_thrust = self.mass_kg * (self.usrCmd_thrust_norm/50)*self.usrCmd_gravity_ms2 # max thrust is always 2x gravity (50% cmd = gravity)
            engine_thrust_bz = self.engine_thrust * math.cos(math.radians(self.usrCmd_thrust_angle_deg)) # thrust along body z-axis
            engine_thrust_bx = self.engine_thrust * math.sin(math.radians(self.usrCmd_thrust_angle_deg)) # thrust along body x-axis

            engine_thrust_z = engine_thrust_bz * math.cos(math.radians(self.phi_deg)) + engine_thrust_bx * math.sin(math.radians(self.phi_deg)) # thrust along inertial z-axis, rotated by current roll attitude
            engine_thrust_x = engine_thrust_bx * math.cos(math.radians(self.phi_deg)) + engine_thrust_bz * math.sin(math.radians(self.phi_deg)) # thrust along inertial x-axis, rotated by current roll attitude

            # integrate z-dof
            self.Az_ms2 = -self.usrCmd_gravity_ms2 + (engine_thrust_z/self.mass_kg)
            self.Vz_ms += self.Az_ms2*self.dt
            self.Pz_m += self.Vz_ms*self.dt

            # integrate x-dof
            self.Ax_ms2 = (engine_thrust_x/self.mass_kg)
            self.Vx_ms += self.Ax_ms2*self.dt
            self.Px_m += self.Vx_ms*self.dt

            # integrate phi-dof
            self.pdot_dps =  (-engine_thrust_bx * self.height_m/2)/self.inertia_kgm2*57.2958 # moment offset of thrust line is 1/2 self.height
            self.p_dps += self.pdot_dps*self.dt
            self.phi_deg += self.p_dps*self.dt
        
        else:
            phi_rad = math.radians(self.phi_deg)
            sinPhi = math.sin(phi_rad)
            cosPhi = math.cos(phi_rad)
            # we are on the left leg
            #lgl_2x = cx_m - (self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*cosPhi - (self.height_m/2)*sinPhi # ground
            if self.phi_deg <= 0:
                if not self.left_leg_first_touch:
                    self.leg_left_x = self.Px_m - (self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*cosPhi - (self.height_m/2)*sinPhi
                    self.left_leg_first_touch = True
                self.Px_m = self.leg_left_x - (-(self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*cosPhi - (self.height_m/2)*sinPhi)
                self.Pz_m = -((self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*sinPhi - (self.height_m/2)*cosPhi)
                dx_m = self.leg_left_x - self.Px_m # offset distance between leg on ground and cg

            # we are on the right leg
            else:
                if not self.right_leg_first_touch:
                    self.leg_right_x = self.Px_m + ((self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*cosPhi - (self.height_m/2)*sinPhi)
                    self.right_leg_first_touch = True
                self.Px_m = self.leg_right_x - ((self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*cosPhi - (self.height_m/2)*sinPhi)
                self.Pz_m = -(-(self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*sinPhi - (self.height_m/2)*cosPhi)
                dx_m = self.leg_right_x - self.Px_m # offset distance between leg on ground and cg

            # this is the first timestep in this new "mode", need to dump linear momentum into angular momentum
            if not self.converted_momentum_2_rotation:
                #TODO do this correctly 
                x_mom_kgms = self.Vx_ms * self.mass_kg * 0.8
                z_mom_kgms = self.Vz_ms * self.mass_kg * 0.8
                self.p_dps += ((x_mom_kgms*self.Pz_m)/self.landed_inertia_kgm2 + (z_mom_kgms*dx_m)/self.landed_inertia_kgm2) * 57.2958
                self.converted_momentum_2_rotation = True
            
            # integrate z-dof - NOPE
            self.Az_ms2 = 0
            self.Vz_ms = 0

            # integrate x-dof - NOPE
            self.Ax_ms2 = 0
            self.Vx_ms = 0

            # integrate phi-dof
            if not self.crashed:
                self.pdot_dps =  -(self.mass_kg*self.usrCmd_gravity_ms2 * dx_m + self.p_dps*100)/self.landed_inertia_kgm2*57.2958
                self.p_dps += self.pdot_dps*self.dt
                self.phi_deg += self.p_dps*self.dt

            # check if we crashed 
            if not self.crashed:
                if self.phi_deg> 100 or self.phi_deg < -100:
                    self.crashed = True


        self.update_viz(self.Pz_m, self.Px_m, self.phi_deg) # update the visualization

        #self.auto_reset() # reset if out of bounds

        # Register callback
        self.root.after(int(self.dt*1000), self.update)

    def touchdown_check(self):
        phi_rad = math.radians(self.phi_deg)
        sinPhi = math.sin(phi_rad)
        cosPhi = math.cos(phi_rad)
        if self.touchdown:
            return
        else:
            leg_left_pz = self.Pz_m + (self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*sinPhi - (self.height_m/2)*cosPhi
            leg_right_pz = self.Pz_m - (self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*sinPhi - (self.height_m/2)*cosPhi

            if leg_left_pz <= 0 or leg_right_pz <= 0:
                self.touchdown = True
                self.TD_Vz_ms = self.Vz_ms
                self.TD_Vx_ms = self.Vx_ms
                self.TD_phi_deg = self.phi_deg


    def auto_reset(self):
        if self.Pz_m < -10 or self.Pz_m > 50 or self.Px_m < -40 or self.Px_m > 40:
            self.initSim()

    def update_viz(self, z, x, phi):
        # Clear previous drawings
        self.canvas.delete("all")

        # text box readouts
        self.canvas.create_text(self.width - 150, 50, text="Alt: "+str(round(self.Pz_m-self.height_m/2,2))+" m", font=("Arial bold", 12), fill="white", anchor='w')
        self.canvas.create_text(self.width - 232, 70, text="Descent Rate: "+str(round(self.Vz_ms,2))+" m/s", font=("Arial bold", 12), fill="white", anchor='w')
        self.canvas.create_text(self.width - 210, 90, text="Lateral Vel: "+str(round(self.Vx_ms,2))+" m/s", font=("Arial bold", 12), fill="white", anchor='w')
        self.canvas.create_text(self.width - 188, 120, text="Attitude: "+str(round(self.phi_deg,2))+" deg", font=("Arial bold", 12), fill="white", anchor='w')
        self.canvas.create_text(self.width - 227, 140, text="Attitude Rate: "+str(round(self.p_dps,2))+" deg/s", font=("Arial bold", 12), fill="white", anchor='w')

        # some stuff
        phi_rad = math.radians(self.phi_deg)
        cosPhi = math.cos(phi_rad)
        sinPhi = math.sin(phi_rad)

        thrust_angle_rad = math.radians(self.usrCmd_thrust_angle_deg + self.phi_deg)
        cosThrAng = math.cos(thrust_angle_rad)
        sinThrAng = math.sin(thrust_angle_rad)

        # draw the ground first
        lander_coords = [self.convert_to_pixel_space(-10000, 0), 
                         self.convert_to_pixel_space(10000, 0), 
                         self.convert_to_pixel_space(10000, -500), 
                         self.convert_to_pixel_space(-10000, -500)]
        self.canvas.create_polygon(lander_coords, fill="gray")

        # center of lander
        cx_m = self.Px_m
        cz_m = self.Pz_m

        # compute 4 corners of rectangular lander
        lander_p1x_m = cx_m - self.width_m/2*cosPhi + self.height_m/2*sinPhi # upper left
        lander_p1z_m = cz_m + self.height_m/2*cosPhi + self.width_m/2*sinPhi # upper left
        lander_p2x_m = cx_m + self.width_m/2*cosPhi + self.height_m/2*sinPhi # upper right
        lander_p2z_m = cz_m + self.height_m/2*cosPhi - self.width_m/2*sinPhi # upper right
        lander_p3x_m = cx_m + self.width_m/2*cosPhi - self.height_m/2*sinPhi # lower right
        lander_p3z_m = cz_m - self.height_m/2*cosPhi - self.width_m/2*sinPhi # lower right
        lander_p4x_m = cx_m - self.width_m/2*cosPhi - self.height_m/2*sinPhi # lower left
        lander_p4z_m = cz_m - self.height_m/2*cosPhi + self.width_m/2*sinPhi # lower left
        # convert lander rectangle pts into pixel space and draw it
        lander_coords = [self.convert_to_pixel_space(lander_p1x_m, lander_p1z_m), 
                         self.convert_to_pixel_space(lander_p2x_m, lander_p2z_m), 
                         self.convert_to_pixel_space(lander_p3x_m, lander_p3z_m), 
                         self.convert_to_pixel_space(lander_p4x_m, lander_p4z_m)]
        self.canvas.create_polygon(lander_coords, fill="green")

        # compute 3 corners of flame plume
        if not self.touchdown:
            flame_length_m = self.height_m*0.05 + self.usrCmd_thrust_norm/100 * 4 # 100% throttle = flames length of lander
            flames_p1x_m = cx_m - self.width_m/2*cosPhi - self.height_m/2*sinPhi # left
            flames_p1z_m = cz_m - self.height_m/2*cosPhi + self.width_m/2*sinPhi # left
            flames_p2x_m = cx_m + self.width_m/2*cosPhi - self.height_m/2*sinPhi # right
            flames_p2z_m = cz_m - self.height_m/2*cosPhi - self.width_m/2*sinPhi # right
            flames_p3x_m = cx_m - (self.height_m/2 + flame_length_m)*sinThrAng # point
            flames_p3z_m = cz_m - (self.height_m/2 + flame_length_m)*cosThrAng# point
            # convert flame triangle pts into pixel space and draw it
            flame_coords = [self.convert_to_pixel_space(flames_p1x_m, flames_p1z_m), 
                            self.convert_to_pixel_space(flames_p2x_m, flames_p2z_m), 
                            self.convert_to_pixel_space(flames_p3x_m, flames_p3z_m)]
            self.canvas.create_polygon(flame_coords, fill="orange")
        else:
            self.canvas.create_text(30, 470, text="Touchdown Vertical Velocity: "+str(round(self.TD_Vz_ms,2))+" m/s", font=("Arial bold", 12), fill="white", anchor='w')
            self.canvas.create_text(360, 470, text="Touchdown Horizontal Velocity: "+str(round(self.TD_Vx_ms,2))+" m/s", font=("Arial bold", 12), fill="white", anchor='w')
            self.canvas.create_text(700, 470, text="Touchdown Attitude: "+str(round(self.TD_phi_deg,2))+" deg", font=("Arial bold", 12), fill="white", anchor='w')

        # compute left landing gear points
        lgl_1x = cx_m - self.width_m/2*cosPhi - (self.height_m/2 - (self.leg_width_m/2 - self.width_m/2)*0.707)*sinPhi # attached to body
        lgl_1y = cz_m - (self.height_m/2 - (self.leg_width_m/2 - self.width_m/2)*0.707)*cosPhi + self.width_m/2*sinPhi # attached to body
        lgl_2x = cx_m - (self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*cosPhi - (self.height_m/2)*sinPhi # ground
        lgl_2y = cz_m + (self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*sinPhi - (self.height_m/2)*cosPhi # ground
        lgl_coords = [self.convert_to_pixel_space(lgl_1x, lgl_1y), 
                      self.convert_to_pixel_space(lgl_2x, lgl_2y)]
        self.canvas.create_line(lgl_coords, fill="green", width=0.25*self.scale_factor, capstyle="round")

        # compute right landing gear points
        lgl_1x = cx_m + self.width_m/2*cosPhi - (self.height_m/2 - (self.leg_width_m/2 - self.width_m/2)*0.707)*sinPhi # attached to body
        lgl_1y = cz_m - (self.height_m/2 - (self.leg_width_m/2 - self.width_m/2)*0.707)*cosPhi - self.width_m/2*sinPhi # attached to body
        lgl_2x = cx_m + (self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*cosPhi - (self.height_m/2)*sinPhi # ground
        lgl_2y = cz_m - (self.width_m/2 + (self.leg_width_m/2 - self.width_m/2)*0.707)*sinPhi - (self.height_m/2)*cosPhi # ground
        lgl_coords = [self.convert_to_pixel_space(lgl_1x, lgl_1y), 
                      self.convert_to_pixel_space(lgl_2x, lgl_2y)]
        self.canvas.create_line(lgl_coords, fill="green", width=0.25*self.scale_factor, capstyle="round")

        if self.draw_cg_forces:
            # draw the cg
            [px,pz] = self.convert_to_pixel_space(cx_m, cz_m)
            self.create_circle(self.canvas, px, pz, 0.2*self.scale_factor)
            # draw the thrust line
            arrLength = self.engine_thrust/5000.0
            tarr_1x = cx_m - 0*self.width_m/2*cosPhi - (self.height_m/2)*sinPhi # attached to body
            tarr_1y = cz_m - (self.height_m/2)*cosPhi + 0*self.width_m/2*sinPhi # attached to body
            tarr_2x = cx_m - (self.height_m/2 + arrLength)*sinThrAng # thrust tip
            tarr_2y = cz_m - (self.height_m/2 + arrLength)*cosThrAng # thrust tip
            tarr_coords = [self.convert_to_pixel_space(tarr_2x, tarr_2y), 
                        self.convert_to_pixel_space(tarr_1x, tarr_1y)]
            if not self.touchdown:
                self.canvas.create_line(tarr_coords, fill="red", width=0.1*self.scale_factor, arrow=tk.LAST, capstyle="round")
            
            # draw the weight line
            arrLength = self.mass_kg*self.usrCmd_gravity_ms2/5000.0
            warr_1x = cx_m # attached to body
            warr_1y = cz_m # attached to body
            warr_2x = cx_m  # thrust tip
            warr_2y = cz_m - arrLength # thrust tip
            warr_coords = [self.convert_to_pixel_space(warr_1x, warr_1y), 
                        self.convert_to_pixel_space(warr_2x, warr_2y)]
            self.canvas.create_line(warr_coords, fill="red", width=0.1*self.scale_factor, arrow=tk.LAST, capstyle="round")


    def create_circle(self, canvas, center_x, center_y, radius, **kwargs):
        x1 = center_x - radius
        y1 = center_y - radius
        x2 = center_x + radius
        y2 = center_y + radius
        return canvas.create_oval(x1, y1, x2, y2,outline="black",fill="white", **kwargs)

    def convert_to_pixel_space(self, x, z):
        # Convert simulation units to pixels using a scale factor
        scale_factor = self.scale_factor
        px = (x + self.width / (2 * scale_factor)) * scale_factor
        pz = self.height*1.4 - (z + self.height / (2 * scale_factor)) * scale_factor  # Invert y-coordinate for canvas
        return px, pz

###########################################################################################################################

def main():
    try:
        root = tk.Tk()
        root.state('zoomed')
        root.title("Lander Simulator")
        width = 1000
        height = 500
        simulator = landerSim(root, width, height)
        root.mainloop()
    except KeyboardInterrupt:
        sys.exit(0)

if __name__ == "__main__":
    main()
