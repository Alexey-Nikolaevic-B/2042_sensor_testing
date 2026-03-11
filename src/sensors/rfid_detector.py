from .detector import SensorDetector, register_detector


@register_detector
class RfidDetector(SensorDetector):
    sensor_type = "rfid"
    priority    = 10

    def detect(self, sdf_content: str) -> bool:
        return "libRFID_tag_plugin.so" in sdf_content