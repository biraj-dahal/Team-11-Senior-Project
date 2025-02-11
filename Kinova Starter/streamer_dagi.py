import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

def on_eos(bus, message, loop):
    """ End of stream callback """
    print('End of Stream')
    loop.quit()

def on_error(bus, message, loop):
    """ Error callback """
    print(f"Error: {message.parse_error()}")
    loop.quit()

def start_rtsp_stream():
    # Initialize GStreamer
    Gst.init(None)

    # Create the pipeline for the RTSP stream
    pipeline = Gst.parse_launch("rtspsrc location=rtsp://10.0.0.222/color ! decodebin ! autovideosink")

    # Set up the loop and bus for handling events
    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message::eos", on_eos, loop)
    bus.connect("message::error", on_error, loop)

    # Start the pipeline
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except:
        pass

    # Stop the pipeline when done
    pipeline.set_state(Gst.State.NULL)

if __name__ == "__main__":
    start_rtsp_stream()
