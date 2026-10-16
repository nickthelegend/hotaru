// secrets.example.h -- copy to secrets.h and fill in. secrets.h is in
// .gitignore so your keys never reach the repository.
#pragma once

#define WIFI_SSID       "YOUR_WIFI_NAME"
#define WIFI_PASS       "YOUR_WIFI_PASSWORD"

#define DEEPGRAM_KEY    "your-deepgram-key"
#define OPENROUTER_KEY  "sk-or-v1-your-openrouter-key"

// GHOST hub (optional): the computer running ghost-nebius/hub. Leave "" to
// keep the lamp standalone. GHOST_CSI 0 turns off Wi-Fi sensing.
#define GHOST_HUB_HOST  ""
#define GHOST_HUB_PORT  8770
