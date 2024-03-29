#pragma once

enum {
    VHFVORADIO_IFACE_CMD_GET_MODE,
    VHFVORADIO_IFACE_CMD_SET_MODE,
    VHFVORADIO_IFACE_CMD_GET_BANDWIDTH,
    VHFVORADIO_IFACE_CMD_SET_BANDWIDTH,
    VHFVORADIO_IFACE_CMD_GET_SQUELCH_ENABLED,
    VHFVORADIO_IFACE_CMD_SET_SQUELCH_ENABLED,
    VHFVORADIO_IFACE_CMD_GET_SQUELCH_LEVEL,
    VHFVORADIO_IFACE_CMD_SET_SQUELCH_LEVEL,
    VHFVORADIO_IFACE_CMD_GET_CTCSS_ENABLED,
    VHFVORADIO_IFACE_CMD_SET_CTCSS_ENABLED,
    VHFVORADIO_IFACE_CMD_GET_CTCSS_TONE,
    VHFVORADIO_IFACE_CMD_SET_CTCSS_TONE,
    VHFVORADIO_IFACE_CMD_GET_DCS_ENABLED,
    VHFVORADIO_IFACE_CMD_SET_DCS_ENABLED,
    VHFVORADIO_IFACE_CMD_GET_DCS_CODE,
    VHFVORADIO_IFACE_CMD_SET_DCS_CODE,
};

enum {
    VHFVORADIO_IFACE_MODE_NFM,
    VHFVORADIO_IFACE_MODE_DSD,
    VHFVORADIO_IFACE_MODE_OLDDSD,
    VHFVORADIO_IFACE_MODE_WFM,
    VHFVORADIO_IFACE_MODE_AM,
    VHFVORADIO_IFACE_MODE_USB,
    VHFVORADIO_IFACE_MODE_LSB,
    VHFVORADIO_IFACE_MODE_RAW
};
