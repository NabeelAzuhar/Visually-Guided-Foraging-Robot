import sensor, time

class Cam(object):
    """
    The Cam class manages the camera sensor for image capturing, processing,
    and color tracking. It initializes the camera parameters and sets the color
    thresholds for blob detection.
    """

    def __init__(self, thresholds, gain = 25):
        """
        Initialise the Cam object by setting up camera parameters and
        configuring color thresholds.
        """
        # Configure camera settings
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.VGA)   # Set frame size to 640x480
        sensor.skip_frames(time=2000)   # Allow the camera to adjust to light levels

        # Both must be turned off for color tracking
        sensor.set_auto_gain(False, gain_db = gain)
        sensor.set_auto_whitebal(False)

        # Initialise sensor properties
        self.w_centre = sensor.width()/2
        self.h_centre = sensor.height()/2
        self.h_fov = 70.8
        self.v_fov = 55.6
        self.camera_elevation_angle = -11
        self.clock = time.clock()

        # Define color tracking thresholds for Red, Green, Blue, and Yellow colors
        # Thresholds are in the order of (L Min, L Max, A Min, A Max, B Min, B Max)
        self.thresholds = thresholds


    def get_blobs(self) -> tuple:
        """
        Capture an image and detect color blobs based on predefined thresholds.

        Returns:
            blobs (list): List of detected blobs.
            img (image): Captured image used to find blobs.
        """
        img = sensor.snapshot()

        blobs = img.find_blobs(self.thresholds,pixels_threshold=60,area_threshold=60)

        return blobs, img


    def get_blobs_bottom(self) -> tuple:
        """
        Capture an image and detect colour blobs based on predefined thresholds.
        Region of interest is set to the bottom 1/2 of the image.

        Returns:
            blobs (list): List of detected blobs.
            img (image): Captured image used to find blobs.
        """
        img = sensor.snapshot()

        blobs = img.find_blobs(self.thresholds,pixels_threshold=150,area_threshold=150,
                               roi=(0,int(sensor.height()/2),
                                    int(sensor.width()),int(2*sensor.height()/3)))

        return blobs, img

    def get_blobs_bottom_thresh(self, blob_id) -> tuple:
        """
        Capture an image and detect colour blobs based on predefined thresholds.
        Region of interest is set to the bottom 1/2 of the image.

        Returns:
            blobs (list): List of detected blobs.
            img (image): Captured image used to find blobs.
        """
        img = sensor.snapshot()

        blobs = img.find_blobs([self.thresholds[blob_id]],pixels_threshold=150,area_threshold=150,
                               roi=(0,int(sensor.height()/2),
                                    int(sensor.width()),int(2*sensor.height()/3)))

        return blobs, img


    def get_biggest_blob(self, blobs):
        """
        Identify and return the largest blob from a list of detected blobs.

        Args:
            blobs (list): List of detected blobs.

        Returns:
            big_blob (blob): The biggest blob from list - see OpenMV docs for blob class.
        """
        max_pixels = 0
        big_blob = None

        for blob in blobs:
            # Update the big blob if the current blob has more pixels
            if blob.pixels() > max_pixels:
                max_pixels = blob.pixels()
                big_blob = blob

        return big_blob


    def get_blob_colours(self, blobs) -> int:
        """
        Returns the binary code (as int) of thresholds met by each blob

        Args:
            blobs (list): List of detected blobs.

        Returns:
            colours (list): List of binary codes (as int) of thresholds met by each element in blobs.
        """
        colours = []

        for blob in blobs:
            colours.append(blob[8])

        return colours


    def find_blob(self, blobs, threshold_idx: int):
        """
        Finds the first blob in blobs that was detected using a specified threshold

        Args:
            blobs (list): List of detected blobs.
            threshold_idx (int): Index along self.thresholds.

        Returns:
            found_idx (int): Index along blobs for the first blob that was detected using self.thresholds(threshold_idx)
        """
        colours = self.get_blob_colours(blobs)

        for found_idx, colour in enumerate(colours):
            if colour == pow(2, threshold_idx):
                return found_idx

        return None

    def filter_blobs(self, blobs, threshold_idxs: list):
        """
        Finds the blobs of the right colours that were detected using a specified threshold

        Args:
            blobs (list): List of detected blobs.
            threshold_idx (int): Index along self.thresholds.

        Returns:
            filtered_blobs (int): Blobs of the right colour that were detected using self.thresholds(threshold_idx)
        """
        colours = self.get_blob_colours(blobs)

        powered_idxs = []
        for idx in threshold_idxs:
            powered_idxs.append(pow(2, idx))


        filtered_blobs = []
        for found_idx, colour in enumerate(colours):
            if colour in powered_idxs:
                filtered_blobs.append(blobs[found_idx])

        return filtered_blobs


if __name__ == "__main__":
    #
    # Blob threshold tester
    #
    # Use this code to determine colour tracking thresholds
    # Edited by Daniel Ko 2024

    import sensor
    import math

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_whitebal(False)  # must be turned off for colour tracking

    # Change gain here to work with the lighting conditions you have
    sensor.set_auto_gain(False, gain_db = 25)  # must be turned off for color tracking
    #

    # Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
    # The below thresholds track in general red/green things. You may wish to tune them...
    thresholds = [(23, 75, 32, -14, -10, -51), #blue
                  (44, 12, 63, 32, 55, 15), #red
                  (37, 58, -54, -30, 21, 49), #green
                 (41, 75, -25, -6, 33, 57), #sun
                 ]


    # Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
    # returned by "find_blobs" below. "roi" sets the region of interest of the image in which to find
    # blobs (x, y, width, height). Currently set to look for blobs in the bottom 2/3 of the image.
    while True:
        img = sensor.snapshot()
        for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150,
                                   roi=(0,int(sensor.height()/2),int(sensor.width()),int(2*sensor.height()/3))):
            img.draw_rectangle(blob.rect())

            if blob.elongation() > 0.5:
                img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_keypoints(
                [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
            )
