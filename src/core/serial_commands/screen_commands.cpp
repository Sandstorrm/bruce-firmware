#include "screen_commands.h"
#include "core/settings.h"
#include "core/utils.h" // time
#include <globals.h>

uint32_t brightnessCallback(cmd *c) {
    // backlight brightness adjust (range 0-255) https://docs.flipper.net/development/cli/#XQQAI
    // e.g. "led br 127"

    Command cmd(c);

    Argument arg = cmd.getArgument("value");
    String strValue = arg.getValue();
    strValue.trim();

    int value = (atoi(strValue.c_str()) * 100) / 255; // convert to 0-100 range
    value = min(max(1, value), 100);

    serialDevice->println("Settings led brightness to " + String(value) + "%");
    setBrightness(value, false);
    return true;
}

uint32_t rgbColorCallback(cmd *c) {
    // change UI color
    // e.g. "screen color rgb 255 255 255"

    Command cmd(c);

    Argument redArg = cmd.getArgument("red");
    Argument greenArg = cmd.getArgument("green");
    Argument blueArg = cmd.getArgument("blue");
    String strRed = redArg.getValue();
    String strGreen = greenArg.getValue();
    String strBlue = blueArg.getValue();
    strRed.trim();
    strGreen.trim();
    strBlue.trim();

    int r = atoi(strRed.c_str());
    int g = atoi(strGreen.c_str());
    int b = atoi(strBlue.c_str());

    if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
        serialDevice->println("Invalid color: " + strRed + " " + strGreen + " " + strBlue);
        return false;
    }

    uint16_t hexColor = tft.color565(r, g, b);
    bruceConfig.priColor = hexColor; // change global var, dont save in config
    return true;
}

uint32_t hexColorCallback(cmd *c) {
    // change UI color
    // e.g. "screen color hex ff00ff"

    Command cmd(c);

    Argument arg = cmd.getArgument("value");
    String strValue = arg.getValue();
    strValue.trim();

    if (strValue.length() % 2 == 1) {
        serialDevice->println("Invalid hex value: " + strValue);
        return false;
    }

    uint32_t value = static_cast<uint32_t>(std::stoul(strValue.c_str(), nullptr, 16));

    if (value > 0xFFFFFF) {
        serialDevice->println("Invalid color: " + strValue);
        return false;
    }

    uint16_t r = (value >> 8) & 0xF800;
    uint16_t g = (value >> 5) & 0x07E0;
    uint16_t b = (value >> 3) & 0x001F;

    uint16_t hexColor = (r | g | b);
    bruceConfig.priColor = hexColor; // change global var, dont save in config
    return true;
}

uint32_t clockCallback(cmd *c) {
#if defined(HAS_RTC)
    updateTimeStr(_rtc.getTimeStruct());
    serialDevice->printf("\nCurrent time: %s", timeStr);
#else
    updateTimeStr(rtc.getTimeStruct());
    serialDevice->printf("\nCurrent time: %s", timeStr);
#endif
    return true;
}

// ---- screen view: what is on the Core2's display, for a remote user who cannot see it ----------------------
static String jsonEsc(const String &s) {
    String o;
    o.reserve(s.length() + 4);
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == '"' || c == '\\') {
            o += '\\';
            o += c;
        } else if ((uint8_t)c < 0x20) {
            o += ' ';
        } else {
            o += c;
        }
    }
    return o;
}

// Same button press the `nav` command makes, without its menu listing.
static void remotePress(volatile bool *var) {
    unsigned long t0 = millis();
    while (millis() <= t0 + 1) {
        if (*var == false) {
            AnyKeyPress = true;
            SerialCmdPress = true;
            *var = true;
            if (!LongPress) break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// One line of JSON: {"mode":"menu","title":..,"sel":N,"options":[..]} or {"mode":"app","lines":[..]}
static String screenViewJson() {
    String j = "{\"w\":" + String((int)tftWidth) + ",\"h\":" + String((int)tftHeight);
    if (uiState == 1 && !options.empty()) {
        int sel = 0;
        String labels;
        for (size_t i = 0; i < options.size(); i++) {
            if (options[i].hovered) sel = (int)i;
            if (i) labels += ",";
            labels += "\"" + jsonEsc(options[i].label) + "\"";
        }
        const char *type = menuOptionType == 0 ? "main" : (menuOptionType == 1 ? "sub" : "menu");
        j += ",\"mode\":\"menu\",\"type\":\"" + String(type) + "\",\"title\":\"" + jsonEsc(menuOptionLabel) +
             "\",\"sel\":" + String(sel) + ",\"options\":[" + labels + "]";
    } else {
        String text = tft.getScreenText(), lines;
        int start = 0;
        bool first = true;
        while (start <= (int)text.length() && text.length()) {
            int nl = text.indexOf('\n', start);
            String line = nl < 0 ? text.substring(start) : text.substring(start, nl);
            line.trim();
            if (line.length()) {
                if (!first) lines += ",";
                lines += "\"" + jsonEsc(line) + "\"";
                first = false;
            }
            if (nl < 0) break;
            start = nl + 1;
        }
        j += ",\"mode\":\"app\",\"title\":\"" + jsonEsc(menuOptionLabel) + "\",\"lines\":[" + lines + "]";
    }
    return j + "}";
}

// screen view                 -> report what is on screen
// screen view open N          -> choose entry N of the menu that is showing, then report
// screen view back|sel|next|prev|up|down -> press that button, then report
uint32_t screenViewCallback(cmd *c) {
    Command cmd(c);
    String action = cmd.getArgument("action").getValue();
    String value = cmd.getArgument("value").getValue();
    action.trim();
    action.toLowerCase();
    value.trim();
    if (action == "open") {
        int n = value.toInt();
        // a menu we just came back to may still be redrawing: give it a moment to start waiting for a choice
        unsigned long w0 = millis();
        while (uiState != 1 && millis() - w0 < 1500) vTaskDelay(20 / portTICK_PERIOD_MS);
        if (uiState == 1 && n >= 0 && n < (int)options.size()) forceMenuOption = n;
    } else if (action == "back" || action == "esc") {
        remotePress(&EscPress);
    } else if (action == "sel" || action == "select") {
        remotePress(&SelPress);
    } else if (action == "next") {
        remotePress(&NextPress);
    } else if (action == "prev") {
        remotePress(&PrevPress);
    } else if (action == "up") {
        remotePress(&UpPress);
    } else if (action == "down") {
        remotePress(&DownPress);
    }
    // the UI runs in another task: wait until the choice was taken up and the screen stopped changing
    unsigned long t0 = millis();
    while (forceMenuOption >= 0 && millis() - t0 < 1500) vTaskDelay(20 / portTICK_PERIOD_MS);
    if (action.length()) {
        // an app can take a second to start (BLE stack, ...): wait until neither the mode nor the text changes any more
        uint8_t last = uiState;
        uint32_t gen = tft.getScreenGen();
        unsigned long stableSince = millis();
        while (millis() - t0 < 4000 && millis() - stableSince < 450) {
            vTaskDelay(25 / portTICK_PERIOD_MS);
            if (uiState != last || tft.getScreenGen() != gen) {
                last = uiState;
                gen = tft.getScreenGen();
                stableSince = millis();
            }
        }
    } else {
        vTaskDelay(60 / portTICK_PERIOD_MS);
    }
    serialDevice->println(screenViewJson());
    return true;
}

void createScreenCommands(SimpleCLI *cli) {
    Command clockCmd = cli->addCommand("clock", clockCallback);

    Command screenCmd = cli->addCompositeCmd("screen");

    Command viewCmd = screenCmd.addCommand("view", screenViewCallback);
    viewCmd.addPosArg("action", "");
    viewCmd.addPosArg("value", "");

    Command brightCmd = screenCmd.addCommand("br/ight/ness", brightnessCallback);
    brightCmd.addPosArg("value");

    Command colorCmd = screenCmd.addCompositeCmd("color");
    Command rgbColorCmd = colorCmd.addCommand("rgb", rgbColorCallback);
    rgbColorCmd.addPosArg("red");
    rgbColorCmd.addPosArg("green");
    rgbColorCmd.addPosArg("blue");
    Command hexColorCmd = colorCmd.addCommand("hex", hexColorCallback);
    hexColorCmd.addPosArg("value");
}
