"""模拟 RFID 缓存关键状态，验证现场故障链路不会回退。"""

from dataclasses import dataclass, field
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RFID_C = ROOT / "User" / "Application" / "Beep" / "sscRFID.c"
HANDLESCAN_C = ROOT / "User" / "Application" / "Handle" / "handlescan.c"


RFID_PAYLOAD_USER = bytes.fromhex("31 28 1A 00 01 01 00 00 0A 00 00 0C 00 00 00 00")


@dataclass
class RfidResult:
    valid: bool = True
    cache_hit: bool = False
    channel: int = 1
    source: int = 2
    payload: bytes = RFID_PAYLOAD_USER
    sequence: int = 0
    presence_sequence: int = 0


@dataclass
class RfidChannelState:
    last_valid: bool = False
    last_sequence: int = 0
    last_presence_sequence: int = 0
    memory_valid: bool = False
    memory_payload: bytes = field(default_factory=bytes)
    result_sequence: int = 0
    presence_sequence: int = 0


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def update_cache(state: RfidChannelState, result: RfidResult, fast_mode: bool = False) -> bool:
    """按 sscRFID.c 的 Rfid_UpdateParsedCache() 关键语义更新状态。"""
    same_payload = state.memory_valid and state.memory_payload == result.payload
    result.cache_hit = same_payload
    state.presence_sequence += 1
    result.presence_sequence = state.presence_sequence

    if same_payload:
        if fast_mode or not state.last_valid:
            state.result_sequence += 1
            result.sequence = state.result_sequence
            state.last_valid = True
            state.last_sequence = result.sequence
            state.last_presence_sequence = result.presence_sequence
            return True

        result.sequence = state.last_sequence
        state.last_presence_sequence = result.presence_sequence
        return False

    state.result_sequence += 1
    result.sequence = state.result_sequence
    state.memory_valid = True
    state.memory_payload = result.payload
    state.last_valid = True
    state.last_sequence = result.sequence
    state.last_presence_sequence = result.presence_sequence
    return True


def clear_current_result(state: RfidChannelState) -> None:
    """按 Rfid_ClearChannelResult() 只清当前可上报结果，不清历史 payload 记忆。"""
    state.last_valid = False
    state.last_sequence = 0
    state.last_presence_sequence = 0
    state.result_sequence = 0
    state.presence_sequence = 0


def main() -> None:
    rfid_c = RFID_C.read_text(encoding="utf-8")
    handlescan_c = HANDLESCAN_C.read_text(encoding="utf-8")

    require("s_payload_memory" in rfid_c, "RFID must keep payload memory separate from current result")
    require("result.cache_hit == false" in handlescan_c,
            "online RFID monitoring must still gate repeated same-payload beep by cache_hit")

    state = RfidChannelState()

    first = RfidResult()
    require(update_cache(state, first, fast_mode=True) is True, "first fast RFID result must publish a business result")
    require(first.cache_hit is False, "first RFID result must be treated as new information")

    same_online = RfidResult()
    require(update_cache(state, same_online, fast_mode=False) is False, "same online payload must not publish a new business result")
    require(same_online.cache_hit is True, "same online payload must be marked as cache hit")
    require(state.last_sequence == first.sequence, "same online payload must keep previous sequence")

    clear_current_result(state)
    require(state.memory_valid is True, "clearing current RFID result must keep remembered payload for no-repeat beep")

    same_reappears = RfidResult()
    require(update_cache(state, same_reappears, fast_mode=True) is True, "same payload after clear must republish so UI can show it again")
    require(same_reappears.cache_hit is True,
            "same payload after clear stays cache-hit for change detection; WAIT_RFID_TOOL may still beep as online feedback")

    changed = RfidResult(payload=bytes.fromhex("32 28 1A 00 01 01 00 00 0A 00 00 0C 00 00 00 00"))
    require(update_cache(state, changed, fast_mode=False) is True, "changed payload must publish a new business result")
    require(changed.cache_hit is False, "changed payload must not be cache hit and may beep once")

    print("ok - RFID state regression passed")


if __name__ == "__main__":
    main()
