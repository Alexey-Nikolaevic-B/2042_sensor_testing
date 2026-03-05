from .sensor_storage import (
    # Original API (kept for backwards compatibility)
    add_sensor,
    delete_sensor,
    get_sensors,

    # New API
    init_db,
    update_sensor,
    get_all_sensors,
    get_sensor_by_name,
    get_sensor_types,
    save_test_result,
    get_latest_test_results,
    get_test_history,
)
