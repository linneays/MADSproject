"""Read a MADS bag (format v1) into a list of (t_ns, topic, json) records.
Payloads are snappy-compressed JSON (MADS `compress = true`)."""
import json, struct, sys
import cramjam

def read_bag(path):
    data = open(path, "rb").read()
    assert data[:4] == b"MBAG", "not a MADS bag"
    ver, flags = struct.unpack_from("<II", data, 4)
    crc = flags & 1
    off, out = 12, []
    while off + 4 <= len(data):
        if data[off:off+4] in (b"MBFT", b"MBTR"):
            break
        (rlen,) = struct.unpack_from("<I", data, off); off += 4
        end = off + rlen
        if end > len(data): break
        (ts,) = struct.unpack_from("<q", data, off); off += 8
        (tl,) = struct.unpack_from("<I", data, off); off += 4
        topic = data[off:off+tl].decode(); off += tl
        (n,) = struct.unpack_from("<I", data, off); off += 4
        parts = []
        for _ in range(n):
            (pl,) = struct.unpack_from("<I", data, off); off += 4
            parts.append(data[off:off+pl]); off += pl
        off = end
        msg = None
        for p in reversed(parts):
            for dec in (lambda b: bytes(cramjam.snappy.decompress_raw(b)), lambda b: b):
                try:
                    msg = json.loads(dec(p)); break
                except Exception:
                    pass
            if msg is not None: break
        out.append((ts, topic, msg, len(parts)))
    return out

if __name__ == "__main__":
    recs = read_bag(sys.argv[1])
    print(len(recs), "records")
