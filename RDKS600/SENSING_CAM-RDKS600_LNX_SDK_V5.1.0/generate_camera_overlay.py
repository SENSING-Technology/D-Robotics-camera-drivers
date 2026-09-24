#!/usr/bin/env python3
"""Interactive S600 camera overlay generator for J12, J14, or all RX ports."""
import argparse, re, shutil, subprocess, sys, tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
NAME = "s600_sensing_cam_rx0123_portABCD_yuv"
SOURCE = HERE.parent / "platform_source_code/source/hobot-camera/v4l2/dts_overlays"
MONO = ("SN2M4EFGD", "SG3S11AFSK", "SHF3L", "SG8-OX08DC-5300-G2G-Hxxx", "SG8S-AR0820C-5300-G2G-Hxxx")
STEREO = ("S36", "S56")
LETTERS = "abcd"
MONO_RESOLUTION = {
    "SN2M4EFGD": (1920, 1080),
    "SG3S11AFSK": (1920, 1536),
    "SHF3L": (1920, 1536),
    "SG8-OX08DC-5300-G2G-Hxxx": (3840, 2160),
    "SG8S-AR0820C-5300-G2G-Hxxx": (3840, 2160),
}
MONO_LINK_RATE = {
    "SN2M4EFGD": 3,
    "SG3S11AFSK": 3,
    "SHF3L": 6,
    "SG8-OX08DC-5300-G2G-Hxxx": 6,
    "SG8S-AR0820C-5300-G2G-Hxxx": 6,
}
TRIGGER_DUTY_NS = 10_000

def template_path():
    for p in (HERE / "overlay" / f"{NAME}.dtso",
              HERE / f"{NAME}.dtso",
              SOURCE / f"{NAME}.dtso"):
        if p.is_file(): return p
    raise FileNotFoundError(f"cannot find overlay/{NAME}.dtso or {NAME}.dtso")

def span(text, name):
    m = re.search(rf"(?m)^\s*(?:[\w-]+\s*:\s*)?{re.escape(name)}\s*\{{", text)
    if not m: raise ValueError(f"node not found: {name}")
    op = text.find("{", m.start(), m.end()); depth = 0; string = False; esc = False
    for i in range(op, len(text)):
        c = text[i]
        if string:
            if esc: esc = False
            elif c == "\\": esc = True
            elif c == '"': string = False
        elif c == '"': string = True
        elif c == "{": depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                e = i + 1
                while e < len(text) and text[e] in " \t": e += 1
                if e < len(text) and text[e] == ";": e += 1
                return m.start(), e
    raise ValueError(f"unterminated node: {name}")

def replace_node(text, name, new):
    b, e = span(text, name); indent = re.match(r"\s*", text[b:]).group(0).split("\n")[-1]
    new = "\n".join(indent + x if x else "" for x in new.splitlines())
    return text[:b] + new + text[e:]

def prop(text, name, key, value):
    b, e = span(text, name); block = text[b:e]
    pat = re.compile(rf"(?m)^(\s*){re.escape(key)}\s*=\s*[^;]+;([^\r\n]*)$")
    if len(pat.findall(block)) != 1: raise ValueError(f"expected one {key} in {name}")
    block = pat.sub(lambda m: f"{m.group(1)}{key} = {value};{m.group(2)}", block, count=1)
    return text[:b] + block + text[e:]

def upsert_prop(text, name, key, value):
    b, e = span(text, name); block = text[b:e]
    pat = re.compile(rf"(?m)^(\s*){re.escape(key)}\s*=\s*[^;]+;([^\r\n]*)$")
    matches = pat.findall(block)
    if len(matches) > 1:
        raise ValueError(f"multiple {key} properties in {name}")
    if matches:
        block = pat.sub(lambda m: f"{m.group(1)}{key} = {value};{m.group(2)}", block, count=1)
    else:
        status = re.search(r'(?m)^(\s*)status\s*=\s*[^;]+;', block)
        if not status: raise ValueError(f"status missing in {name}")
        block = block[:status.start()] + f"{status.group(1)}{key} = {value};\n" + block[status.start():]
    return text[:b] + block + text[e:]

def remove_prop(text, name, key):
    b, e = span(text, name); block = text[b:e]
    pat = re.compile(rf"(?m)^\s*{re.escape(key)}\s*=\s*[^;]+;[^\r\n]*(?:\r?\n)?")
    block, count = pat.subn("", block)
    if count > 1: raise ValueError(f"multiple {key} properties in {name}")
    return text[:b] + block + text[e:]

def labeled_prop(text, label, key, value):
    m = re.search(rf"(?m)^\s*{re.escape(label)}\s*:\s*[\w@,-]+\s*\{{", text)
    if not m: raise ValueError(f"labeled node not found: {label}")
    op = text.find("{", m.start(), m.end()); depth = 0
    for i in range(op, len(text)):
        if text[i] == "{": depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                block = text[m.start():i + 1]
                pat = re.compile(rf"(?m)^(\s*){re.escape(key)}\s*=\s*[^;]+;")
                if len(pat.findall(block)) != 1:
                    raise ValueError(f"expected one {key} in {label}")
                block = pat.sub(lambda x: f"{x.group(1)}{key} = {value};", block, count=1)
                return text[:m.start()] + block + text[i + 1:]
    raise ValueError(f"unterminated labeled node: {label}")

def labeled_remove_prop(text, label, key):
    m = re.search(rf"(?m)^\s*{re.escape(label)}\s*:\s*[\w@,-]+\s*\{{", text)
    if not m: raise ValueError(f"labeled node not found: {label}")
    op = text.find("{", m.start(), m.end()); depth = 0
    for i in range(op, len(text)):
        if text[i] == "{": depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                block = text[m.start():i + 1]
                pat = re.compile(rf"(?m)^\s*{re.escape(key)}\s*=\s*[^;]+;[^\r\n]*(?:\r?\n)?")
                block, count = pat.subn("", block)
                if count > 1: raise ValueError(f"multiple {key} properties in {label}")
                return text[:m.start()] + block + text[i + 1:]
    raise ValueError(f"unterminated labeled node: {label}")

def model_prop(text, name, model):
    b, e = span(text, name); block = text[b:e]
    cm = re.search(r"(?m)^(\s*)compatible\s*=\s*[^;]+;[^\r\n]*$", block)
    if not cm: raise ValueError(f"compatible missing: {name}")
    block = block[:cm.end()] + f'\n{cm.group(1)}sensing,model = "{model}";' + block[cm.end():]
    return text[:b] + block + text[e:]

def choose(rx, port, remaining_streams, allow_stereo):
    choices = ["DISABLED"]
    if remaining_streams:
        choices += list(MONO)
    if remaining_streams >= 2 and allow_stereo:
        choices += list(STEREO)
    print(f"\nRX{rx} Port {'ABCD'[port]}:")
    for i, m in enumerate(choices):
        kind = "unused physical link" if m == "DISABLED" else ("stereo, one physical link and two VCs" if m in STEREO else "mono YUV, one VC")
        print(f"  {i}: {m} ({kind})")
    while True:
        try: model = choices[int(input(f"Select 0-{len(choices)-1}: ").strip())]; break
        except (ValueError, IndexError): print("Invalid selection.")
    if model == "DISABLED": return model, 0, 0
    if model == "S36": return model, 1920, 1536
    if model == "S56": return model, 2560, 1984
    width, height = MONO_RESOLUTION[model]
    print(f"  Fixed configuration: {width}x{height}, {MONO_LINK_RATE[model]}G")
    return model, width, height

def selections(active_rx):
    all_rx = [None] * 4
    for rx in range(4):
        if rx not in active_rx:
            continue
        row = []; used_streams = 0; p = 0
        while p < 4:
            c = choose(rx, p, 4 - used_streams, p in (0, 2))
            row.append(c)
            used_streams += 2 if c[0] in STEREO else (0 if c[0] == "DISABLED" else 1)
            if c[0] in STEREO:
                row.append(("RESERVED", 0, 0))
                print(f"  {c[0]} uses RX{rx} Port {'ABCD'[p]}+{'ABCD'[p+1]}, reserves output VC{used_streams-2}+VC{used_streams-1}, and skips Port {'ABCD'[p+1]}.")
                p += 2
            else:
                p += 1
        all_rx[rx] = row
    return all_rx

def fragment_status(text, number, value):
    b, e = span(text, f"fragment@{number}"); block = text[b:e]
    pat = re.compile(r'(?m)^(\s*)status\s*=\s*[^;]+;')
    if not pat.search(block):
        raise ValueError(f"status missing in fragment@{number}")
    block = pat.sub(lambda m: f'{m.group(1)}status = "{value}";', block, count=1)
    return text[:b] + block + text[e:]

def sensor_name(rx, p): return f"rx{rx}_sensing_{LETTERS[p]}@{0x11+p:02x}"
def ser_name(rx, p): return f"rx{rx}_max9295_{LETTERS[p]}@{0x41+p:02x}"

def stereo_node(rx, port, camera, vcs, trigger_period_ns):
    model, w, h = camera
    vc0, vc1 = vcs
    alias_base = 0x22 + port
    ep = LETTERS[port]
    x = [f"rx{rx}_{model.lower()}@{alias_base:02x} {{", f'\tcompatible = "sensing,{model.lower()}-gmsl2";',
         f"\treg = <0x{alias_base:02x}>;", f"\tlink-port = <{port}>;", "\tgmsl-link-rate = <6>;",
         f"\twidth = <{w}>;", f"\theight = <{h}>;"]
    defaults = "0x1a 0x1b" if model == "S36" else "0x10 0x11"
    x += [f"\tsensing,sensor-default-addresses = <{defaults}>;",
          f"\tsensing,sensor-alias-addresses = <0x{alias_base:02x} 0x{alias_base+1:02x}>;",
          f"\tsensing,serializer-alias = <0x{0x41+port:02x}>;",
          f"\tsensing,stream-pipes = <{vc0} {vc1}>;",
          f"\tsensing,stream-dst-vcs = <{vc0} {vc1}>;"]
    if model in STEREO:
        x += ["\tsensing,trigger-mode = <1>;", "\tsensing,trigger-mfp = <7>;",
              "\tsensing,trigger-gpio-id = <7>;", f"\tsensing,trigger-period-ns = <{trigger_period_ns}>;",
              f"\tsensing,trigger-duty-ns = <{TRIGGER_DUTY_NS}>;"]
    x += [f"\td-robotics,serdes-dser-device = <&rx{rx}_dser>;", '\tstatus = "okay";', "", "\tports {",
          "\t\t#address-cells = <1>;", "\t\t#size-cells = <0>;", "\t\tport@0 {", "\t\t\treg = <0>;",
          f"\t\t\trx{rx}_sensor_{ep}_out_0: endpoint {{", f"\t\t\t\tremote-endpoint = <&rx{rx}_mipi_vc{vc0}_in>;",
          f"\t\t\t\tvirtual-channel = <{vc0}>;", "\t\t\t};", "\t\t};", "\t\tport@1 {", "\t\t\treg = <1>;",
          f"\t\t\trx{rx}_sensor_{ep}_out_1: endpoint {{", f"\t\t\t\tremote-endpoint = <&rx{rx}_mipi_vc{vc1}_in>;",
          f"\t\t\t\tvirtual-channel = <{vc1}>;", "\t\t\t};", "\t\t};", "\t};", "};"]
    return "\n".join(x)

def generate(text, cams, trigger_period_ns):
    pwm = ("&lpwm0 0", "&lpwm0 2", "&lpwm3 0", "&lpwm3 2")
    for rx, row in enumerate(cams):
        if row is None:
            # Keep the CIM and MIPI infrastructure enabled. The vendor
            # vid_v4l2 probe iterates every matching platform node and aborts
            # if any one is disabled. Disabling the RX I2C fragment is enough
            # to suppress Sensor/SerDes clients on an unused connector.
            text = fragment_status(text, rx, "disabled")
            continue
        d = f"rx{rx}_max96712@29"
        text = upsert_prop(text, d, "sensing,trigger-period-ns", f"<{trigger_period_ns}>")
        text = upsert_prop(text, d, "sensing,trigger-duty-ns", f"<{TRIGGER_DUTY_NS}>")
        text = prop(text, d, "pwms", f"<{pwm[rx]} {trigger_period_ns} {TRIGGER_DUTY_NS}>")
        next_vc = 0
        used_vcs = set()
        for p, (model, w, h) in enumerate(row):
            n = sensor_name(rx, p)
            if model in ("DISABLED", "RESERVED"):
                text = prop(text, ser_name(rx, p), "status", '"disabled"')
                text = prop(text, n, "status", '"disabled"')
                text = remove_prop(text, n, "remote-endpoint")
                continue
            if model in STEREO:
                text = prop(text, ser_name(rx, p), "status", '"disabled"')
                text = replace_node(text, n, stereo_node(rx, p, row[p], (next_vc, next_vc + 1), trigger_period_ns))
                text = labeled_prop(text, f"rx{rx}_mipi_vc{next_vc}_in", "remote-endpoint", f"<&rx{rx}_sensor_{LETTERS[p]}_out_0>")
                text = labeled_prop(text, f"rx{rx}_mipi_vc{next_vc+1}_in", "remote-endpoint", f"<&rx{rx}_sensor_{LETTERS[p]}_out_1>")
                used_vcs.update((next_vc, next_vc + 1))
                if model in STEREO:
                    d = f"rx{rx}_max96712@29"
                    text = prop(text, d, "pwms", f"<{pwm[rx]} {trigger_period_ns} {TRIGGER_DUTY_NS}>")
                    text = prop(text, d, "trig_mode", "<0>")
                next_vc += 2
            else:
                text = prop(text, ser_name(rx, p), "status", '"okay"')
                text = prop(text, n, "status", '"okay"')
                text = prop(text, n, "width", f"<{w}>")
                text = prop(text, n, "height", f"<{h}>")
                text = prop(text, n, "gmsl-link-rate", f"<{MONO_LINK_RATE[model]}>")
                # The mono sensing driver routes serializer/deserializer pipe
                # and destination VC from the physical GMSL link index.  Keep
                # Port A/B/C/D on VC0/1/2/3 even when earlier ports are disabled.
                vc = p
                text = prop(text, n, "remote-endpoint", f"<&rx{rx}_mipi_vc{vc}_in>")
                text = prop(text, n, "virtual-channel", f"<{vc}>")
                text = labeled_prop(text, f"rx{rx}_mipi_vc{vc}_in", "remote-endpoint", f"<&rx{rx}_sensor_{LETTERS[p]}_out>")
                text = model_prop(text, n, model)
                used_vcs.add(vc)
                next_vc += 1
        for vc in range(4):
            if vc not in used_vcs:
                text = labeled_remove_prop(text, f"rx{rx}_mipi_vc{vc}_in", "remote-endpoint")
    return text

def compile_dtso(src, dst):
    cpp, dtc = shutil.which("cpp"), shutil.which("dtc")
    if not cpp or not dtc: raise RuntimeError("cpp and dtc are required")
    inc = next((p for p in (SOURCE.parents[2] / "kernel/include", Path("/usr/include")) if (p/"dt-bindings/gpio/gpio.h").is_file()), None)
    with tempfile.TemporaryDirectory() as td:
        expanded = Path(td)/"x.dts"; args = [cpp, "-nostdinc", "-undef", "-D__DTS__", "-x", "assembler-with-cpp"]
        if inc: args += ["-I", str(inc)]
        with expanded.open("wb") as f:
            rc = subprocess.run(args+[str(src)], stdout=f,
                                stderr=subprocess.DEVNULL if not inc else None).returncode
        if rc:
            raw = src.read_text(); raw = re.sub(r"(?m)^\s*#include\s+<dt-bindings/gpio/gpio.h>\s*$", "", raw).replace("GPIO_ACTIVE_HIGH", "0")
            expanded.write_text(raw)
        subprocess.run([dtc, "-@", "-I", "dts", "-O", "dtb", "-o", str(dst), str(expanded)], check=True)

def main():
    modes = {"j12": (0, 1), "j14": (2, 3), "all": (0, 1, 2, 3)}
    parser = argparse.ArgumentParser(
        description="Generate an S600 Sensing camera DT overlay.")
    parser.add_argument("mode", nargs="?", default="all", choices=tuple(modes),
                        help="connector group to enable (default: all)")
    parser.add_argument("--fps", type=int, choices=(20, 25, 30), default=25,
                        help="S36/S56 external-trigger frame rate (default: 25)")
    args = parser.parse_args()
    mode = args.mode
    active_rx = modes[mode]
    trigger_period_ns = round(1_000_000_000 / args.fps)
    try:
        tpl = template_path(); print(f"Template: {tpl}\nMode: {mode} (enabled RX: {', '.join(f'RX{x}' for x in active_rx)})\nS36/S56 external trigger: {args.fps} Hz (period {trigger_period_ns} ns, duty {TRIGGER_DUTY_NS} ns)\nStereo S36/S56 consumes two adjacent physical links and two output VCs.\nEach RX supports at most four output streams.\nMonocular resolutions are fixed by the generator.")
        cams = selections(active_rx)
        output_dir = HERE / "overlay"
        output_dir.mkdir(parents=True, exist_ok=True)
        out = output_dir / f"{NAME}_generated.dtso"
        dtbo = output_dir / f"{NAME}_generated.dtbo"
        out.write_text(generate(tpl.read_text(), cams, trigger_period_ns), encoding="utf-8"); compile_dtso(out, dtbo)
        print("\nFinal configuration:")
        for rx,row in enumerate(cams):
            if row is None:
                print(f"  RX{rx}: disabled ({'J12' if rx < 2 else 'J14'} not selected)")
                continue
            vc = 0
            for p,c in enumerate(row):
                if c[0] in ("DISABLED", "RESERVED"):
                    state = "reserved by previous stereo camera" if c[0] == "RESERVED" else "disabled"
                    print(f"  RX{rx} Port {'ABCD'[p]}: {state}")
                    continue
                vcs = f"VC{vc}+VC{vc+1}" if c[0] in STEREO else f"VC{p}"
                print(f"  RX{rx} Port {'ABCD'[p]} {vcs}: {c[0]} {c[1]}x{c[2]}")
                vc += 2 if c[0] in STEREO else 1
        print(f"\nGenerated: {out}\nGenerated: {dtbo}\n/boot/overlays was not modified."); return 0
    except (EOFError, KeyboardInterrupt): print("\nCancelled.", file=sys.stderr); return 130
    except Exception as e: print(f"error: {e}", file=sys.stderr); return 1
if __name__ == "__main__": sys.exit(main())
