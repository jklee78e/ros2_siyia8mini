#!/usr/bin/env python3
import sys, os, time, re
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

def on_message(bus, message, loop):
    """ 
    [디버깅 모드] 
    모든 GStreamer 버스 메시지의 상세 내용을 파싱하여 출력합니다.
    """
    t = message.type

    if t == Gst.MessageType.EOS:
        print("\n--- 스트림 끝 (EOS) 도달 --- 종료합니다.")
        pipeline.set_state(Gst.State.NULL)
        loop.quit()
        
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f"\n🚨🚨 에러: {err} (디버그: {debug}) 🚨🚨")
        pipeline.set_state(Gst.State.NULL)
        loop.quit()
        
    elif t == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        print(f"\n⚠️ 경고: {err} (디버그: {debug})")
        
    elif t == Gst.MessageType.STREAM_START:
        print("\n--- 스트림 시작됨 ---")
        
    elif t == Gst.MessageType.STATE_CHANGED:
        # 상태 변경 메시지 (너무 많아서 생략)
        pass
    
    # --- 상세 로그가 출력되던 메시지들 ---
    
    elif t == Gst.MessageType.TAG:
        taglist = message.parse_tag()
        # 태그 리스트의 모든 태그를 순회하며 출력
        print(f"[TAG] from {message.src.name}:")
        def print_tag(taglist, tag, userdata):
            print(f"  > {Gst.tag_get_nick(tag)}: {taglist.get_string(tag)[1]}")
        taglist.foreach(print_tag, None)

    elif t == Gst.MessageType.QOS:
        format, processed, dropped = message.parse_qos_stats()
        print(f"[QOS] from {message.src.name}: Format={format.value_nick}, Processed={processed}, Dropped={dropped}")

    elif t == Gst.MessageType.LATENCY:
        print(f"[LATENCY] from {message.src.name}: 레이턴시 정보가 업데이트되었습니다.")

    elif t == Gst.MessageType.ASYNC_DONE:
        print(f"[ASYNC_DONE] from {message.src.name}: 비동기 작업 완료.")
        
    elif t == Gst.MessageType.NEW_CLOCK:
        print(f"[NEW_CLOCK] from {message.src.name}: 새 클럭이 제공되었습니다.")
        
    elif t == Gst.MessageType.ELEMENT:
        print(f"[ELEMENT] from {message.src.name}: 엘리먼트 메시지: {message.get_structure().to_string()}")

    elif t == Gst.MessageType.PROGRESS:
        type_nick = message.parse_progress_type().value_nick
        code = message.parse_progress_code()
        text = message.parse_progress_text()
        print(f"[PROGRESS] from {message.src.name}: Type={type_nick}, Code={code}, Text={text}")
        
    else:
        # 위에서 처리하지 않은 나머지 모든 메시지
        print(f"[OTHER] Type: {t.value_nick} (Source: {message.src.name})")

    return True

if __name__ == '__main__':
    Gst.init(None)

    # H.264 파이프라인 (decodebin3 사용)
    pipeline_str = """
        rtspsrc location=rtsp://192.168.144.25:8554/main.264 
        latency=0 udp-reconnect=1 timeout=0 do-retransmission=false ! 
        decodebin3 ! 
        queue max-size-buffers=1 leaky=2 ! 
        videoconvert ! 
        autovideosink
    """
    
    print(f"GStreamer 파이프라인 시작 중...\n{pipeline_str}")
    
    try:
        pipeline = Gst.parse_launch(pipeline_str)
    except GLib.Error as e:
        print(f"파이프라인 생성 실패: {e}")
        sys.exit(1)

    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    
    bus.add_signal_watch()
    bus.connect("message", on_message, loop)

    pipeline.set_state(Gst.State.PLAYING)
    print("파이프라인 실행 중... (Ctrl+C로 종료)")

    try:
        loop.run()
    except KeyboardInterrupt:
        print("\n(Ctrl+C 수신) 파이프라인 종료 중...")
        pipeline.set_state(Gst.State.NULL)
        loop.quit()

    print("Bye bye ;)")