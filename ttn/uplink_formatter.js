function readUint16LE(bytes, index) {
  return bytes[index] | (bytes[index + 1] << 8);
}

function readInt16LE(bytes, index) {
  const value = readUint16LE(bytes, index);
  return value > 0x7fff ? value - 0x10000 : value;
}

function readUint32LE(bytes, index) {
  return (
    bytes[index] |
    (bytes[index + 1] << 8) |
    (bytes[index + 2] << 16) |
    (bytes[index + 3] << 24)
  ) >>> 0;
}

function readInt32LE(bytes, index) {
  const value = readUint32LE(bytes, index);
  return value > 0x7fffffff ? value - 0x100000000 : value;
}

function decodeUplink(input) {
  const bytes = input.bytes;

  // Support both the original 16-byte payload and the new 18-byte deep-sleep payload
  if (bytes.length !== 16 && bytes.length !== 18) {
    return {
      errors: ["Expected 16 or 18 byte payload, got " + bytes.length]
    };
  }

  const batt_mv = readUint16LE(bytes, 0);
  const lat_e7 = readInt32LE(bytes, 2);
  const lon_e7 = readInt32LE(bytes, 6);
  const alt_m = readInt16LE(bytes, 10);
  const gps_unix = readUint32LE(bytes, 12);

  // New field in 18-byte payload: time-to-first-fix in deciseconds
  const ttff_ds = bytes.length >= 18 ? readUint16LE(bytes, 16) : null;

  return {
    data: {
      batt_mv,
      lat_e7,
      lon_e7,
      alt_m,
      gps_unix,
      latitude: lat_e7 / 1e7,
      longitude: lon_e7 / 1e7,
      has_fix: lat_e7 !== 0 || lon_e7 !== 0,
      ttff_ds: ttff_ds,
      ttff_sec: ttff_ds !== null ? ttff_ds / 10.0 : null
    }
  };
}
