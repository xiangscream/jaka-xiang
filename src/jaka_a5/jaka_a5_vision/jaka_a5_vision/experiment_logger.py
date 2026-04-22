import csv
import os
from typing import Optional


class CsvExperimentLogger:
    FIELDNAMES = [
        'wall_time',
        'sim_time_sec',
        'node',
        'cycle_id',
        'state',
        'event',
        'detail',
        'stage_duration_sec',
        'target_fresh',
        'servo_enabled',
        'battery_attached',
    ]

    def __init__(self, path: str) -> None:
        self.path = path
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            with open(path, 'w', newline='', encoding='utf-8') as handle:
                writer = csv.DictWriter(handle, fieldnames=self.FIELDNAMES)
                writer.writeheader()

    def log_event(
        self,
        *,
        node: str,
        sim_time_sec: float,
        cycle_id: int,
        state: str,
        event: str,
        detail: str,
        stage_duration_sec: Optional[float] = None,
        target_fresh: Optional[bool] = None,
        servo_enabled: Optional[bool] = None,
        battery_attached: Optional[bool] = None,
    ) -> None:
        with open(self.path, 'a', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=self.FIELDNAMES)
            writer.writerow(
                {
                    'wall_time': self._wall_time(),
                    'sim_time_sec': f'{sim_time_sec:.3f}',
                    'node': node,
                    'cycle_id': cycle_id,
                    'state': state,
                    'event': event,
                    'detail': detail,
                    'stage_duration_sec': self._format_optional(stage_duration_sec),
                    'target_fresh': self._format_optional(target_fresh),
                    'servo_enabled': self._format_optional(servo_enabled),
                    'battery_attached': self._format_optional(battery_attached),
                }
            )

    def _wall_time(self) -> str:
        import datetime

        return datetime.datetime.now(datetime.timezone.utc).isoformat()

    @staticmethod
    def _format_optional(value: Optional[object]) -> str:
        if value is None:
            return ''
        if isinstance(value, bool):
            return 'true' if value else 'false'
        if isinstance(value, float):
            return f'{value:.3f}'
        return str(value)
