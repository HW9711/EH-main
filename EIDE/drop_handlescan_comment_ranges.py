from pathlib import Path


TARGET = Path(r"D:\Study_HL\F413EXOsSSCH_RTOSV1.5\User\Application\Handle\handlescan.c")


def main() -> None:
    lines = TARGET.read_text(encoding="utf-8-sig").splitlines()

    # 当前文件里残留的乱码注释都是“旧注释块 + 新注释块”叠在一起，
    # 这里按已经定位好的固定行段删除旧块，只保留下面正常显示的中文注释。
    remove_ranges = [
        (295, 301),
        (355, 361),
        (415, 421),
        (432, 438),
        (449, 455),
        (467, 473),
        (485, 491),
    ]

    keep = []
    for lineno, line in enumerate(lines, 1):
        should_drop = any(start <= lineno <= end for start, end in remove_ranges)
        if not should_drop:
            keep.append(line)

    TARGET.write_text("\n".join(keep) + "\n", encoding="utf-8-sig")


if __name__ == "__main__":
    main()
