import tkinter as tk

# thanks chatgpt

class Joystick:
    def __init__(self, size, root):
        self.size = size
        self.root = root
        self.cursor_sf = 0.1

        # Create a Tkinter window
        #self.root = tk.Tk()
        #self.root.title("Joystick Window")

        # Create a canvas
        self.canvas = tk.Canvas(self.root, width=size, height=size, bg="white")
        self.canvas.pack()

        # Create the joystick handle
        self.handle = self.canvas.create_oval(self.size/2 - size*self.cursor_sf/2, self.size/2 - size*self.cursor_sf/2, self.size/2 + size*self.cursor_sf/2, self.size/2 + size*self.cursor_sf/2, fill="blue")
        self.x = 0
        self.y = 0

        # Bind mouse events to handle movement
        self.canvas.bind("<B1-Motion>", self.move_handle)
        self.canvas.bind("<ButtonRelease-1>", self.reset_handle)

    def move_handle(self, event):
        x, y = event.x, event.y
        # Ensure the handle stays within the canvas bounds
        x = max(min(x, self.size), 0)
        y = max(min(y, self.size), 0)
        # Move the handle to the new position
        self.canvas.coords(self.handle, x, y, x + self.size*self.cursor_sf, y + self.size*self.cursor_sf)
        # Normalize coordinates to range [0, 1]
        norm_x = x / self.size
        norm_y = y / self.size
        self.x = (norm_x - 0.5) * 2
        self.y = -(norm_y - 0.5) * 2

    def reset_handle(self, event):
        # Reset the handle to the center of the canvas
        handle_size = self.size * self.cursor_sf
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        self.canvas.coords(self.handle,
                        canvas_width / 2 - handle_size / 2,
                        canvas_height / 2 - handle_size / 2,
                        canvas_width / 2 + handle_size / 2,
                        canvas_height / 2 + handle_size / 2)
        self.x = 0
        self.y = 0



# Create a JoystickWindow instance
#joystick = Joystick(200, on_joystick_move)
#joystick.run()
