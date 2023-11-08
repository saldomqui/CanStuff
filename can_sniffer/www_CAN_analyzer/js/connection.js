let con;



class Connection {
    constructor() {
        const color_list = [[0, 0, 255], [0, 255, 255], [255, 255, 0], [255, 0, 0]],
            PLOT_REFRESH_PERIOD = 0.2;

        let connection,
            onOffMsgPub,
            selMsgPub,
            varDataPub,
            canSub = 0,
            selMsgSub = 0,
            maxFreq = 0,
            maxFreqChange = 0,
            maxDlc = 0,
            all_can_div,
            min_freq, max_freq,
            readGlobalCheck = true,
            allMsgsCheckVal = true,
            selectedMsgDlc = 0,
            msg_info_div,
            msg_info_div_array = [],
            sel_msg_freq_div,
            sel_msg_freq_change_div,
            msg_sel_prev = 0,
            raw_value_div,
            converted_value_div,
            valuePlot,
            lastPlotTime = 0,
            initTime = 0,
            offset_val_div,
            scale_val_div,
            data_plot_buffer = [],
            msgInspectionPaused = false,
            get_CurrentVarListSrvClient,
            currVariableList = [],
            lastSelMsgVarInfo = {
                name: "",
                msg_id: "",
                bits: [],
                offset: 0.0,
                scale: 1.0,
                freq: 0
            };

        function canvasContext(id, plot_config) {
            let canvas;
            canvas = document.getElementById(id);
            if (canvas != null) {
                return canvas.getContext('2d');
            }
            else
                return null;
        }

        function showDialog(id) {
            let diag = document.getElementById(id);
            if (diag != null) {
                diag.style.display = "block";
            }
        }

        function hideDialog(id) {
            let diag = document.getElementById(id);
            if (diag != null) {
                diag.style.display = "none";
            }
        }

        function getHeatMapColorFromValue(value) {
            let aux, idx1, // |-- Our desired color will be between these two indexes in "color".
                idx2, // |
                fractBetween = 0, // Fraction between "idx1" and "idx2" where our value is.
                num_base_colors, color;
            num_base_colors = color_list.length;
            if (value <= 0) {
                idx1 = 0;
                idx2 = 0;
            }
            else if (value >= 1) {
                idx1 = (num_base_colors - 2);
                idx2 = (num_base_colors - 1);
                fractBetween = 1.0;
            }
            else {
                aux = value * (num_base_colors - 1); // Will multiply value by number of basic colors.
                idx1 = Math.floor(aux); // Our desired color will be after this index.
                idx2 = idx1 + 1; // ... and before this index (inclusive).
                fractBetween = aux - idx1; // Distance between the two indexes (0-1).
            }
            let red = (color_list[idx2][0] - color_list[idx1][0]) * fractBetween + color_list[idx1][0];
            let green = (color_list[idx2][1] - color_list[idx1][1]) * fractBetween + color_list[idx1][1];
            let blue = (color_list[idx2][2] - color_list[idx1][2]) * fractBetween + color_list[idx1][2];
            //console.log("idx1:"+idx1+" idx2:"+idx2+"frac:"+fractBetween+" red:"+red+" green:"+green+" blue:"+blue);
            color = "rgb(" + red + "," + green + "," + blue + ")";
            return color;
        }

        function decimalToHex(d, padding) {
            let hex = Number(d).toString(16);
            padding = typeof (padding) === "undefined" || padding === null ? padding = 2 : padding;
            while (hex.length < padding) {
                hex = "0" + hex;
            }
            return hex;
        }

        function appendValueToTableRow(value) {
            let cell = document.createElement("td"); // Add cell
            let div = document.createElement("div");
            div.style.width = "100px";
            div.style.height = "40px";
            div.style.backgroundColor = "green";
            div.style.textAlign = "center";
            div.innerHTML = decimalToHex(value, 2);
            cell.appendChild(div);
            return cell;
        }

        function plotValue(tstamp, value) {
            let currTime = tstamp.secs + tstamp.nsecs * 1E-9;
            if (lastPlotTime === 0)
                initTime = tstamp.secs + tstamp.nsecs * 1E-9;
            let data_pair = [(currTime - initTime).toFixed(2), value.toFixed(2)];
            data_plot_buffer.push(data_pair);
            if (currTime - lastPlotTime > PLOT_REFRESH_PERIOD) {

                value_plot_config.options.scales.yAxes[0].ticks.min = parseInt(document.getElementById('min_plot_val_id').value);
                value_plot_config.options.scales.yAxes[0].ticks.max = parseInt(document.getElementById('max_plot_val_id').value);

                if (value > value_plot_config.options.scales.yAxes[0].ticks.max) value_plot_config.options.scales.yAxes[0].ticks.max = value;
                if (value < value_plot_config.options.scales.yAxes[0].ticks.min) value_plot_config.options.scales.yAxes[0].ticks.min = value;
                data_plot_buffer.forEach((stored_data) => {
                    value_plot_config.data.labels.push(stored_data[0]);
                    value_plot_config.data.datasets[0].data.push(stored_data[1]);
                });
                while (value_plot_config.data.labels.length > 100) {
                    value_plot_config.data.labels.splice(0, 1);
                    value_plot_config.data.datasets[0].data.splice(0, 1);
                }
                valuePlot.update(0);
                data_plot_buffer = [];
                lastPlotTime = currTime;
            }
        }

        function decodeSelectedBits(data) {
            let value = 0, mask, data_val;
            for (let byte_idx = 0; byte_idx < msg_info_div_array.length; byte_idx++) {
                mask = 0x80;
                data_val = data[byte_idx];
                if (data_val < 0)
                    data_val = 256 + data_val;
                //console.log("b:" + byte_idx +":" + "0x" + decimalToHex(data_val, 2));
                for (let bit_idx = 1; bit_idx < 9; bit_idx++) {
                    if (msg_info_div_array[byte_idx][bit_idx].value === 1) { //selected
                        //console.log("bit:"+(8-bit_idx)+" of byte:"+byte_idx+" selected");
                        value *= 2;
                        if (data_val & mask)
                            value += 1;
                    }
                    mask = mask >> 1;
                }
            }
            return value;
            //console.log("value:0x" + decimalToHex(value, 8));
        }

        function getSelectedBitsList() {
            lastSelMsgVarInfo.bits = [];

            for (let byte_idx = 0; byte_idx < msg_info_div_array.length; byte_idx++) {
                let mask = 0x80;
                let pair = [byte_idx, 0x00];

                for (let bit_idx = 1; bit_idx < 9; bit_idx++) {
                    let bit_num = (8 - bit_idx);

                    if (msg_info_div_array[byte_idx][bit_idx].value === 1) { //selected
                        pair[1] |= mask;
                    }
                    mask = mask >> 1;
                }

                if (pair[1]) {
                    lastSelMsgVarInfo.bits.push(pair);
                    //console.log("byte:" + byte_idx + " has bits selected: mask:" + decimalToHex(pair[1], 2));
                }
            }
        }

        function convertValue(value) {
            let offset, scale;
            offset = parseFloat(offset_val_div.value);
            scale = parseFloat(scale_val_div.value);
            return (value - offset) * scale;
        }

        this.ResetBitChanges = function () {
            console.log("Resetting bit changes");
            for (let byte_idx = 0; byte_idx < msg_info_div_array.length; byte_idx++) {
                for (let bit_idx = 1; bit_idx < msg_info_div_array[byte_idx].length; bit_idx++) {
                    msg_info_div_array[byte_idx][bit_idx].className = "bit_not_changed_cell";
                    for (let bit_idx = 1; bit_idx < 9; bit_idx++) {
                        msg_info_div_array[byte_idx][bit_idx].value = 0;
                    }
                }
            }
        };

        this.ReturnMainMenu = function () {
            hideDialog("msg_inspection");
            showDialog("main_menu");
            selMsgSub.unsubscribe();
            //subscribeAllCANMsgTopic();
            msgInspectionPaused = false;
        };

        this.PauseAllCANData = function () {
            let butt = document.getElementById('bt_pause_all_can_data');
            if (butt.value === "Pause") {
                //console.log("Pausing data");
                butt.value = "Continue";
                canSub.unsubscribe();
            }
            else {
                //console.log("Continuing data");
                butt.value = "Pause";
                subscribeAllCANMsgTopic();
            }
        };

        this.PauseMsgInspection = function () {
            let butt = document.getElementById('button_pause_msg_inspect');
            if (butt.value === "Pause") {
                //console.log("Pausing data");
                butt.value = "Continue";
                msgInspectionPaused = true;
            }
            else {
                //console.log("Continuing data");
                butt.value = "Pause";
                msgInspectionPaused = false;
            }
        };

        this.setCandidateAs = function () {
            let bit_masks = [];

            lastSelMsgVarInfo.name = document.getElementById('variable_candidate').value;

            console.log("Variable name:" + lastSelMsgVarInfo.name);
            getSelectedBitsList();

            console.log("msg_id:" + decimalToHex(lastSelMsgVarInfo.msg_id, 4) + " num selected bytes:" + lastSelMsgVarInfo.bits.length);

            for (let byte = 0; byte < lastSelMsgVarInfo.bits.length; byte++) {
                let bit_mask = new ROSLIB.Message({ "byte_num": lastSelMsgVarInfo.bits[byte][0], "mask": lastSelMsgVarInfo.bits[byte][1] });
                bit_masks.push(bit_mask);
                console.log("sel bit mask on byte:" + lastSelMsgVarInfo.bits[byte][0] + " is:" + decimalToHex(lastSelMsgVarInfo.bits[byte][1], 2));
            }

            lastSelMsgVarInfo.offset = parseFloat(offset_val_div.value);
            lastSelMsgVarInfo.scale = parseFloat(scale_val_div.value);

            console.log("offset:" + lastSelMsgVarInfo.offset + " scale:" + lastSelMsgVarInfo.scale);
            console.log("frequency:" + lastSelMsgVarInfo.freq);

            if (bit_masks.length) {
                let comment = prompt("Do you want to add a comment?", "");

                if (comment === null || comment === "") {
                    console.log("User cancelled the prompt.");
                }

                let varDataMsg = new ROSLIB.Message({ "name": lastSelMsgVarInfo.name, "msg_id": parseInt(lastSelMsgVarInfo.msg_id), "bit_masks": bit_masks, "freq": lastSelMsgVarInfo.freq, "offset": lastSelMsgVarInfo.offset, "scale": lastSelMsgVarInfo.scale, "comment": comment });
                varDataPub.publish(varDataMsg);

                //Update the list of variables
                getCurrentVarList();
            } else {
                alert("You have not selected any bit to generate the mask !!!!");
            }
        };

        function updateButtonMsgInspectPause() {
            let butt = document.getElementById('button_pause_msg_inspect');
            if (msgInspectionPaused == true) {
                //console.log("Pausing data");
                butt.value = "Continue";
            }
            else {
                //console.log("Continuing data");
                butt.value = "Pause";
            }
        }

        function bitSelectCallback(bit_id) {
            let byte_idx = Math.floor(bit_id / 8);
            let bit_idx = bit_id % 8;
            if (msg_info_div_array[byte_idx][8 - bit_idx].value === 1) {
                msg_info_div_array[byte_idx][8 - bit_idx].classList.remove("bit_selected_cell");
                msg_info_div_array[byte_idx][8 - bit_idx].value = 0;
            }
            else {
                msg_info_div_array[byte_idx][8 - bit_idx].classList.add("bit_selected_cell");
                msg_info_div_array[byte_idx][8 - bit_idx].value = 1;
            }
            //console.log("Clicked on bit_id:"+bit_id+ " byte_idx:" + byte_idx + " bit:" + bit_idx);
        }

        function bitSelect(byte_idx, bit_idx, value) {
            if (value === 0) {
                msg_info_div_array[byte_idx][8 - bit_idx].classList.remove("bit_selected_cell");
            }
            else {
                msg_info_div_array[byte_idx][8 - bit_idx].classList.add("bit_selected_cell");
                console.log("Selecting: byte_idx:" + byte_idx + " bit:" + bit_idx);
            }
            msg_info_div_array[byte_idx][8 - bit_idx].value = value;
        }

        this.selectVariable = function (nextElementSibling, value) {
            //console.log("changed to variable:" + value);
            nextElementSibling.value = value;

            //Deselect all selected bits
            //console.log("curr msg num bytes:" + msg_info_div_array.length);
            for (let byte = 0; byte < msg_info_div_array.length; byte++) {
                //console.log("byte:" + byte +" num bits:" + msg_info_div_array[byte].length);
                for (let bit = 0; bit < (msg_info_div_array[byte].length - 1); bit++) {
                    //console.log("Deselecting: byte" + byte + " bit:" + bit);
                    bitSelect(byte, bit, 0);
                }
            }

            currVariableList.forEach(variable => {
                //console.log("var_name:"+variable.name);
                if (variable.name === value) {
                    //console.log("Found var_name:"+value);
                    variable.data.forEach(data => {
                        //console.log("lastSelectedMsgId:"+lastSelMsgVarInfo.msg_id + " dataId:" + data.msg_id);
                        if (lastSelMsgVarInfo.msg_id == data.msg_id) {
                            console.log("Found: variable:" + variable.name + " msg_id:" + data.msg_id + " offset:" + data.offset + " scale:" + data.scale);
                            offset_val_div.value = data.offset.toFixed(0);
                            scale_val_div.value = data.scale.toFixed(5);
                            //                        offset_val_div.value=data.offset;
                            //                        scale_val_div.value=data.scale;
                            //console.log("num bit masks:"+data.bit_masks.length);
                            data.bit_masks.forEach(mask => {
                                let mask_sel_bit = 0x80;
                                console.log("Byte:" + mask.byte_num + " mask:0x" + decimalToHex(mask.mask, 2));
                                for (let bit_idx = 7; bit_idx >= 0; bit_idx--) {
                                    console.log("mask_sel_bit:0x" + decimalToHex(mask_sel_bit, 2) + " mask:0x" + decimalToHex(mask.mask, 2));

                                    if (mask_sel_bit & mask.mask)
                                        bitSelect(mask.byte_num, bit_idx, 1);
                                    else
                                        bitSelect(mask.byte_num, bit_idx, 0);
                                    mask_sel_bit = mask_sel_bit >> 1;
                                }
                            });
                        }
                    });
                }
            });
        }

        function getCurrentVarList() {
            let getCurrVarListSrvRequest = new ROSLIB.ServiceRequest({});

            currVariableList = [];

            get_CurrentVarListSrvClient.callService(getCurrVarListSrvRequest, function (msg) {
                let var_select = document.getElementById("var_select");

                //console.log("Requested variables names list");

                //Remove existing options
                while (var_select.length) {
                    var_select.remove(0);
                }
                let option = document.createElement("option");
                option.value = "";
                option.text = "";
                var_select.add(option);
                var_select.selectedIndex = 0;

                msg.variables.forEach(element => {
                    let variable = { name: element.name, data: [] };
                    element.data.forEach(data => {
                        let msg_id_data = { msg_id: data.msg_id, offset: data.offset, scale: data.scale, bit_masks: [] };
                        //console.log(msg_id_data);
                        data.bit_masks.forEach(mask => {
                            let mask_data = { byte_num: mask.byte_num, mask: mask.mask };
                            msg_id_data.bit_masks.push(mask_data);
                        });

                        variable.data.push(msg_id_data);
                    });
                    currVariableList.push(variable);

                    let option = document.createElement("option");
                    option.value = element.name;
                    option.text = element.name;
                    var_select.add(option);
                    var_select.selectedIndex = 0;

                });
            });
        }

        function inspectMsg(msg_id) {
            lastSelMsgVarInfo.msg_id = msg_id;
            lastSelMsgVarInfo.freq = 0;

            //console.log("clickec button:" + butt_id);
            hideDialog("main_menu");
            showDialog("msg_inspection");

            getCurrentVarList();

            msg_info_div = document.getElementById('msg_info');
            let div_sel_msg_title = document.getElementById("sel_msg_id");
            div_sel_msg_title.innerHTML = "Inspecting Msg ID:0x" + decimalToHex(msg_id, 4);
            selectedMsgDlc = 0;
            msgInspectionPaused = false;
            updateButtonMsgInspectPause();

            let selMsg = new ROSLIB.Message({ "data": parseInt(msg_id) });
            selMsgPub.publish(selMsg);
            if (selMsgSub === 0) {
                selMsgSub = new ROSLIB.Topic({
                    ros: connection,
                    name: '/can/selected_msg',
                    messageType: 'can_sniffer/CanMsgId'
                });
            }
            selMsgSub.subscribe(function (msg) {
                let mask;
                //console.log("selected msg info comming");
                if (selectedMsgDlc != msg.dlc) {
                    console.log("creating DOM structure for selected msg. Num bytes:" + msg.dlc);
                    let table = document.createElement("table"),
                        tbody = document.createElement("tbody"),
                        div_byte_num,
                        div_hex_val,
                        div_byte,
                        div_bit,
                        cell,
                        row;

                    while (msg_info_div.hasChildNodes()) {
                        msg_info_div.removeChild(msg_info_div.lastChild);
                    }
                    msg_info_div_array = [];
                    //console.log(maxDlc);
                    for (let d = 0; d < msg.dlc; d++) { //All bytes in the message
                        if ((d % 4) === 0) {
                            row = document.createElement("tr");
                            tbody.append(row);
                        }
                        cell = document.createElement("td"); // Add cell
                        div_byte = document.createElement("div");
                        div_byte.className = "msg_byte_div";
                        div_byte_num = document.createElement("div");
                        div_byte_num.className = "msg_byte_num_div";
                        div_byte_num.innerHTML = "Byte" + d;
                        div_byte.appendChild(div_byte_num);
                        let bit_array = [];
                        div_hex_val = document.createElement("div");
                        div_hex_val.className = "msg_hex_val_div";
                        div_byte.appendChild(div_hex_val);
                        bit_array.push(div_hex_val);
                        for (let bit = 7; bit >= 0; bit--) {
                            div_bit = document.createElement("div");
                            div_bit.classList.add("bit_not_changed_cell");
                            div_bit.id = d * 8 + bit;
                            div_bit.addEventListener("click", function () { bitSelectCallback(this.id); });
                            div_bit.value = "0";
                            bit_array.push(div_bit);
                            div_byte.appendChild(div_bit);
                        }
                        msg_info_div_array.push(bit_array);
                        cell.appendChild(div_byte);
                        row.appendChild(cell);
                    }
                    tbody.append(row);
                    table.appendChild(tbody);
                    msg_info_div.appendChild(table);
                    selectedMsgDlc = msg.dlc;
                }

                lastSelMsgVarInfo.freq = msg.freq;

                if (msgInspectionPaused == false) {
                    //console.log("Filling up DOM structure");
                    /*------------ HERE FILL UP DOM WITH DATA ---------*/
                    sel_msg_freq_div.innerHTML = "Frequency:" + msg.freq + " Hz";
                    sel_msg_freq_change_div.innerHTML = "Freq. change:" + msg.freq_change + " Hz";
                    if (msg.freq) {
                        let frac = msg.freq / maxFreq;
                        //console.log("freq:"+msg.id_list[i].freq+" frac:"+frac);
                        sel_msg_freq_div.style.backgroundColor = getHeatMapColorFromValue(frac);
                    }
                    else
                        sel_msg_freq_div.style.backgroundColor = "grey";
                    if (msg.freq_change) {
                        sel_msg_freq_change_div.style.backgroundColor = "orange";
                    }
                    else
                        sel_msg_freq_change_div.style.backgroundColor = "grey";
                    for (let byte = 0; byte < msg.dlc; byte++) {
                        let hex_val = "0x";
                        mask = 0x80;
                        if (msg.data[byte] >= 0)
                            hex_val += decimalToHex(msg.data[byte], 2);
                        else
                            hex_val += (decimalToHex(256 + msg.data[byte], 2));
                        //console.log("byte:" + byte + "-->" + hex_val);
                        msg_info_div_array[byte][0].innerHTML = hex_val;
                        for (let bit = 7; bit >= 0; bit--) {
                            if (mask & msg.data[byte]) {
                                msg_info_div_array[byte][8 - bit].innerHTML = "1";
                            }
                            else {
                                msg_info_div_array[byte][8 - bit].innerHTML = "0";
                            }
                            if (msg_sel_prev) {
                                let has_changed = compareBitPosition(msg.data[byte], msg_sel_prev.data[byte], mask);
                                if (has_changed) {
                                    //msg_info_div_array[byte][8 - bit].style.backgroundColor = "red";
                                    msg_info_div_array[byte][8 - bit].classList.remove("bit_not_changed_cell");
                                    msg_info_div_array[byte][8 - bit].classList.add("bit_changed_cell");
                                }
                            }
                            mask = mask >> 1;
                            //console.log("bit:"+bit+" mask:"+ mask);
                        }
                    }
                    /*----------- HERE DECODE SELECTED BITS ---------*/
                    let value = decodeSelectedBits(msg.data);
                    raw_value_div.innerHTML = value;
                    let converted_value = convertValue(value);
                    converted_value_div.innerHTML = converted_value.toFixed(3);
                    plotValue(msg.stamp, converted_value);
                    msg_sel_prev = msg;
                }
            });
            //canSub.unsubscribe();
        }

        function compareBitPosition(byte1, byte2, mask) {
            let b1, b2;
            if (byte1 < 0)
                byte1 = 256 + byte1;
            if (byte2 < 0)
                byte2 = 256 + byte2;
            if (((byte1 ^ byte2) & 0xFF) & mask) {
                return true;
            }
            else {
                return false;
            }
        }

        function allMsgsCheckCallback(is_checked) {
            let checkbox_msgs = document.querySelectorAll("input[type='checkbox']");
            //console.log("clicked all_msgs_checkbox. Is checked;" + is_checked);
            allMsgsCheckVal = !is_checked;
            checkbox_msgs.forEach(element => {
                element.checked = allMsgsCheckVal;
            });
            readGlobalCheck = true;
        }

        function msgCheckButtonCallback(msg_id, is_checked) {
            let onOffMsg = new ROSLIB.Message({ "id": parseInt(msg_id), "active": is_checked });
            //console.log("checked msg id:" + msg_id + " checked:" + is_checked);
            onOffMsgPub.publish(onOffMsg);
        }

        //---------------------------------------------------------------------------------
        function createConnection(ip_adress) {
            console.log("ROS connecting to: " + ip_adress + "...");
            return new ROSLIB.Ros({ url: 'ws://' + ip_adress + ':9090' });
        }

        function advertiseTopics() {
            onOffMsgPub = new ROSLIB.Topic({
                ros: connection,
                name: '/can/on_off_msg',
                messageType: 'can_sniffer/OnOffMsg'
            });
            selMsgPub = new ROSLIB.Topic({
                ros: connection,
                name: '/can/select_msg',
                messageType: 'std_msgs/Int16'
            });
            varDataPub = new ROSLIB.Topic({
                ros: connection,
                name: '/can/inspected_variable',
                messageType: 'can_sniffer/CanVariableData'
            });
        }
        
        function declareServices() {
            //Declaring set run mode client
            get_CurrentVarListSrvClient = new ROSLIB.Service({
                ros: connection,
                name: '/can/get_var_list',
                serviceType: 'can_sniffer/GetVarList'
            });
        }
        function subscribeAllCANMsgTopic() {
            if (canSub)
                canSub.unsubscribe();
            else {
                canSub = new ROSLIB.Topic({
                    ros: connection,
                    name: '/can/all',
                    messageType: 'can_sniffer/CanMsgIdList'
                });
            }
            all_can_div = document.getElementById('all_can_ids');
            min_freq = document.getElementById('can_all_filter_freq_min_id').value;
            max_freq = document.getElementById('can_all_filter_freq_max_id').value;
            canSub.subscribe(function (msg) {
                let table = document.createElement("table"),
                    tbody = document.createElement("tbody"),
                    row = document.createElement("tr"),
                    div,
                    cell,
                    check_all,
                    check,
                    butt;

                /* ---------- ALL MSGS ACTIVATION CHECK -----------*/
                cell = document.createElement("td"); // Add cell
                check_all = document.createElement("input");
                check_all.id = "check_butt_all";
                check_all.type = "checkbox";
                check_all.checked = allMsgsCheckVal;
                check_all.className = "all_can_table_check_cell";
                check_all.addEventListener('change', function () { allMsgsCheckCallback(allMsgsCheckVal); });
                cell.appendChild(check_all);
                row.appendChild(cell);
                //console.log(maxDlc);
                for (let d = 1; d < (maxDlc + 2); d++) {
                    cell = document.createElement("td"); // Add cell
                    div = document.createElement("div");
                    div.className = "all_can_table_data_cell";
                    div.style.backgroundColor = "white";
                    if (d > 1)
                        div.innerHTML = "Byte" + (d - 2);
                    else
                        div.innerHTML = "";
                    cell.appendChild(div);
                    row.appendChild(cell);
                }
                tbody.append(row);
                //console.log("CAN messages comming. num messages:"+msg.id_list.length);
                for (let i = 0; i < msg.id_list.length; i++) {
                    let is_checked = false;
                    if (msg.id_list[i].dlc > maxDlc)
                        maxDlc = msg.id_list[i].dlc;
                    if (msg.id_list[i].freq > maxFreq)
                        maxFreq = msg.id_list[i].freq;
                    if (msg.id_list[i].freq_change > maxFreqChange)
                        maxFreqChange = msg.id_list[i].freq_change;
                    if (readGlobalCheck === true) { //Force reading current state of the check boxes of every msg
                        let checkbox_elem_id = "check_butt_" + msg.id_list[i].id;
                        //console.log("checkbox:"+checkbox_elem_id);
                        let elem_checkbox = document.getElementById("check_butt_" + msg.id_list[i].id);
                        if (elem_checkbox != null) {
                            is_checked = elem_checkbox.checked;
                        }
                        else {
                            is_checked = allMsgsCheckVal;
                        }
                        msgCheckButtonCallback(msg.id_list[i].id, is_checked);
                    }
                    else
                        is_checked = msg.id_list[i].active;
                    if ((msg.id_list[i].freq <= max_freq) && (msg.id_list[i].freq >= min_freq) && (msg.id_list[i].active)) {
                        row = document.createElement("tr");
                        div = document.createElement("div");
                        /* ---------- ACTIVATION CHECK -----------*/
                        cell = document.createElement("td"); // Add cell
                        check = document.createElement("input");
                        check.id = "check_butt_" + msg.id_list[i].id;
                        check.value = msg.id_list[i].id;
                        check.type = "checkbox";
                        check.checked = is_checked;
                        check.className = "all_can_table_check_cell";
                        check.addEventListener('change', function () { msgCheckButtonCallback(this.value, this.checked); });
                        cell.appendChild(check);
                        row.appendChild(cell);
                        /* ---------- CAN_ID -----------*/
                        cell = document.createElement("td"); // Add cell
                        div.id = "can_id_" + msg.id_list[i].id;
                        div.innerHTML = "ID:" + decimalToHex(msg.id_list[i].id, 4);
                        div.className = "all_can_table_id_cell";
                        div.style.backgroundColor = "black";
                        div.style.color = "white";
                        div.style.fontSize = "20px";
                        cell.appendChild(div);
                        row.appendChild(cell);
                        /*------------ CAN_DATA ---------------*/
                        for (let d = 0; d < msg.id_list[i].dlc; d++) {
                            //console.log("d:"+d+" val:"+msg.id_list[i].data[d]);
                            cell = document.createElement("td"); // Add cell
                            div = document.createElement("div");
                            div.className = "all_can_table_data_cell";
                            div.style.fontSize = "20px";
                            if (msg.id_list[i].freq && check.checked) {
                                let frac = msg.id_list[i].freq / maxFreq;
                                //console.log("freq:"+msg.id_list[i].freq+" frac:"+frac);
                                div.style.backgroundColor = getHeatMapColorFromValue(frac);
                            }
                            else
                                div.style.backgroundColor = "grey";
                            if (msg.id_list[i].data[d] >= 0)
                                div.innerHTML = decimalToHex(msg.id_list[i].data[d], 2);
                            else
                                div.innerHTML = decimalToHex(256 + msg.id_list[i].data[d], 2);
                            cell.appendChild(div);
                            row.appendChild(cell);
                        }
                        /*------------ DUMMY EMPTY CELLS ----------------*/
                        for (let d = msg.id_list[i].dlc; d < maxDlc; d++) {
                            //console.log("d:"+d+" val:"+msg.id_list[i].data[d]);
                            cell = document.createElement("td"); // Add cell
                            div = document.createElement("div");
                            div.className = "all_can_table_data_cell";
                            div.style.backgroundColor = "lightgrey";
                            div.innerHTML = "";
                            cell.appendChild(div);
                            row.appendChild(cell);
                        }
                        /*--------------- FREQUENCY -----------------*/
                        cell = document.createElement("td"); // Add cell
                        div = document.createElement("div");
                        div.style.backgroundColor = "black";
                        div.className = "all_can_table_freq_cell";
                        div.style.color = "white";
                        div.innerHTML = "Freq:" + msg.id_list[i].freq + "Hz";
                        cell.appendChild(div);
                        row.appendChild(cell);

                        /*--------------- FREQUENCY oF CHANGE -----------------*/
                        cell = document.createElement("td"); // Add cell
                        div = document.createElement("div");
                        div.className = "all_can_table_freq_change_cell";
                        if (check.checked) {
                            if (msg.id_list[i].freq_change === 1)
                                div.style.backgroundColor = "orange";
                            else if (msg.id_list[i].freq_change === 0)
                                div.style.backgroundColor = "grey";
                            else
                                div.style.backgroundColor = "red";
                        }
                        else {
                            div.style.backgroundColor = "grey";
                        }
                        div.style.color = "black";
                        div.innerHTML = "Freq_change:" + msg.id_list[i].freq_change + "Hz";
                        cell.appendChild(div);
                        row.appendChild(cell);

                        /*--------------- FREQUENCY -----------------*/
                        cell = document.createElement("td"); // Add cell
                        div = document.createElement("div");
                        div.className = "var_names_cell";
                        for (let vn = 0; vn < msg.id_list[i].variables.length; vn++) {
                            div.innerHTML += msg.id_list[i].variables[vn];
                            if (vn < msg.id_list[i].variables.length - 1) div.innerHTML += ", ";
                        }
                        cell.appendChild(div);
                        row.appendChild(cell);

                        /*------------ INSPECT BUTTON ----------------*/
                        cell = document.createElement("td"); // Add cell
                        butt = document.createElement("input");
                        butt.id = msg.id_list[i].id;
                        butt.type = "button";
                        butt.value = "inspect";
                        butt.addEventListener("click", function () { inspectMsg(this.id); });
                        cell.appendChild(butt);
                        row.appendChild(cell);
                        tbody.append(row);
                    }
                }
                table.appendChild(tbody);
                while (all_can_div.hasChildNodes()) {
                    all_can_div.removeChild(all_can_div.lastChild);
                }
                all_can_div.appendChild(table);
                if (readGlobalCheck)
                    readGlobalCheck = false;
                //console.log(all_can_div.innerHTML);
                /*
                /* ---------- HERE FILL UP SECTION CAN MESSAGES HTML SECTION ----------*/
                /*
                                '<div>' +
                    '<td><input type=button type="button" id="bt_select" onclick="setParkingSlot();" style="width:215px; height:40px; font-size:10px;" value="Set as parking slot"></input></td>' +
        
                    '</div>'
                 */
            });
        }

        function register_ROS() {
        }

        this.initIP = function () {
            let button_connect = document.getElementById('connect_butt');
            button_connect.value = "connect";
            let ctx_value = canvasContext('value_canvas');
            if (ctx_value !== null)
                valuePlot = new Chart(ctx_value, value_plot_config);
            hideDialog("msg_inspection");
            showDialog("main_menu");
        };

        this.connect = function () {
            this.connection = createConnection(document.location.hostname); // Create connection
            connection = this.connection;
            connection.on('connection', function () {
                let connectForm = document.getElementById('connect_form'), connectButt = document.getElementById('connect_butt'), ipAddrEntry = document.getElementById('ip_adr_value');
                connectButt.value = "disconnect";
                connectButt.style.background = "#0F0";
                connectForm.setAttribute('action', 'javascript:con.disconnect();');
                register_ROS();

                //let img_div = document.createElement("img"); // Add table
                sel_msg_freq_div = document.getElementById('sel_msg_freq_div');
                sel_msg_freq_change_div = document.getElementById('sel_msg_freq_change_div');
                raw_value_div = document.getElementById('raw_value_div');
                converted_value_div = document.getElementById('converted_value_div');
                offset_val_div = document.getElementById('offset_value_id');
                scale_val_div = document.getElementById('scale_value_id');
                advertiseTopics();
                subscribeAllCANMsgTopic();
                declareServices();

                window.addEventListener('unload', function () {
                    console.log("Closing page");
                }, false);
            });
            // Connection is closed
            connection.on('close', function () {
                console.log("Close connection");
            });
        };
        this.disconnect = function () {
            console.log("Disconnecting from ROS...");
            let connectForm = document.getElementById('connect_form'), connectButt = document.getElementById('connect_butt');
            connectButt.value = "connect";
            connectButt.style.background = "#F00";
            connectForm.setAttribute('action', 'javascript:con.connect();');
            if (canSub) {
                canSub.unsubscribe();
            }
        };
    }
}

if (window.addEventListener) {
    window.addEventListener('load', function () {
        con = new Connection();
        con.initIP();
    }, false);
}
