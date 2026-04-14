function on_init()
	local Bt_data = uart_get_baudrate()
    -- 如果返回值为空或为0，则默认设置为9600
    if Bt_data == nil or Bt_data == 0 then
        uart_set_baudrate(9600)
        Bt_data = 9600
    end
end

function on_control_notify(screen,control,value)
	if screen == 0 then
		if control == 1 and value == 1 then
			set_image(screen,9,"1.jpg")		--设置图片文件
		elseif control == 2 and value == 1 then
			set_image(screen,9,"2.jpg")		--设置图片文件
		elseif control == 3 and value == 1 then
			set_image(screen,9,"3.jpg")		--设置图片文件
        elseif control == 4 and value == 1 then
			set_image(screen,9,"4.jpg")		--设置图片文件
		elseif control == 5 and value == 1 then
			set_image(screen,9,"5.jpg")		--设置图片文件
        elseif control == 6 and value == 1 then
			set_image(screen,9,"6.jpg")		--设置图片文件
        end
	end
end

-- 解析水位状态字节，更新控件颜色
local function parse_water_level(status_byte)
    -- bit0：清水箱 (1=正常绿, 0=缺水红)
    if (status_byte & 0x01) == 0x01 then
        set_back_color(0, 3, 0x07E0)  -- 绿色
    else
        set_back_color(0, 3, 0xF800)  -- 红色
    end

    -- bit1：污水箱 (0=正常绿, 1=满了红)
    if (status_byte & 0x02) == 0x02 then
        set_back_color(0, 2, 0xF800)  -- 红色
    else
        set_back_color(0, 2, 0x07E0)  -- 绿色
    end
end

function on_uart_recv_data(packet, port)
    local hex_str = ""
    local data_bytes = {}
    local leng = 0
    local start_idx = 0

    -- 自动检测下标起点（0 或 1）
    if packet[1] ~= nil and packet[0] == nil then
        start_idx = 1
    end

    -- 收集所有字节
    local i = start_idx
    while packet[i] ~= nil do
        leng = leng + 1
        local byte = packet[i]
        table.insert(data_bytes, byte)
        hex_str = hex_str .. string.format("%02X ", byte)
        i = i + 1
    end

    hex_str = hex_str:gsub("%s+$", "")
    set_text(0, 7, hex_str)

    -- ① 8字节：温度帧
    if leng == 8 then
        if  data_bytes[1] == 0xEE and
            data_bytes[2] == 0x10 and
            data_bytes[3] == 0x00 and
            data_bytes[4] == 0x01 and
            data_bytes[5] == 0x00 and
            data_bytes[6] == 0x01 then

            local high = data_bytes[7]
            local low  = data_bytes[8]
            local raw  = high * 256 + low
            if raw >= 0x8000 then raw = raw - 0x10000 end
            local temp = raw / 10.0
            set_text(2, 10, string.format("%.1f°C", temp))
        end

    -- ② 6字节：单独水位响应帧
    elseif leng == 6 then
        if  data_bytes[1] == 0x01 and
            data_bytes[2] == 0x02 and
            data_bytes[3] == 0x01 then
            parse_water_level(data_bytes[4])
        end

    -- ③ 14字节：水位查询回显(8) + 水位响应(6) 粘包
    -- 加了20ms间隔后温度帧单独成包，这里只剩查询回显+响应
    elseif leng == 14 then
        if  data_bytes[1] == 0x01 and
            data_bytes[2] == 0x02 and
            data_bytes[3] == 0x00 and
            data_bytes[4] == 0x00 and
            data_bytes[5] == 0x00 and
            data_bytes[6] == 0x04 and
            data_bytes[7] == 0x79 and
            data_bytes[8] == 0xC9 then

            -- 验证水位响应帧头
            if  data_bytes[9]  == 0x01 and
                data_bytes[10] == 0x02 and
                data_bytes[11] == 0x01 then
                parse_water_level(data_bytes[12])
            end
        end

    -- ④ 22字节：三帧全部粘包（备用，间隔不够时触发）
    elseif leng == 22 then
        if  data_bytes[9]  == 0x01 and
            data_bytes[10] == 0x02 and
            data_bytes[11] == 0x00 and
            data_bytes[12] == 0x00 and
            data_bytes[13] == 0x00 and
            data_bytes[14] == 0x04 and
            data_bytes[15] == 0x79 and
            data_bytes[16] == 0xC9 then

            if  data_bytes[17] == 0x01 and
                data_bytes[18] == 0x02 and
                data_bytes[19] == 0x01 then
                parse_water_level(data_bytes[20])
            end
        end

    else
        set_text(0, 8, "接收长度错误: " .. tostring(leng))
    end
end
