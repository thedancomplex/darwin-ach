#define DARWIN_LOFARO_ACH 1

#include "lofaro_includes_ach.h"
#include "lofaro_utils.h"
#include "lofaro_types.h"
#include <cstring>
#include <chrono>
#include <functional>
#include <memory>
#include <string>



class DarwinAchClient
{
  public:
    DarwinAchClient();
    int cmd(int cmd);
    int cmd(int cmd, bool block);
    int sleep(double val);

    /* Update Methods */
    int getState();

    /* Data types */
    darwin_data_def_t darwin_ref;
    darwin_data_def_t darwin_state;
    darwin_cmd_def_t  darwin_cmd;
    darwin_cmd_def_t  darwin_cmd_return;

  private:
    LofaroUtils* lu = new LofaroUtils();

    bool run_loop = false;

    /* Reference Channel */
    ach_channel_t chan_darwin_ref;  

    /* State Feedback Channel */
    ach_channel_t chan_darwin_state;

    /* Command channel */
    ach_channel_t chan_darwin_cmd;

    /* Command Channel Return */
    ach_channel_t chan_darwin_cmd_return;
};

DarwinAchClient::DarwinAchClient()
{
  /* Zero Data */
  memset(&this->darwin_ref,          0, sizeof(this->darwin_ref));
  memset(&this->darwin_state,        0, sizeof(this->darwin_state));
  memset(&this->darwin_cmd,          0, sizeof(this->darwin_cmd));
  memset(&this->darwin_cmd_return,   0, sizeof(this->darwin_cmd_return));

  /* Make Ach Channels */
  ach_status_t r = ACH_OK;

  /* Open Channels */
  r = ach_open(&this->chan_darwin_ref,        DARWIN_ACH_CHAN_REF,        NULL);
  r = ach_open(&this->chan_darwin_state,      DARWIN_ACH_CHAN_STATE,      NULL);
  r = ach_open(&this->chan_darwin_cmd,        DARWIN_ACH_CHAN_CMD,        NULL);
  r = ach_open(&this->chan_darwin_cmd_return, DARWIN_ACH_CHAN_CMD_RETURN, NULL);

  /* Do initial put on the channel to make sure the exist */
  ach_put(&this->chan_darwin_ref,        &this->darwin_ref,        sizeof(this->darwin_ref));
  ach_put(&this->chan_darwin_state,      &this->darwin_state,      sizeof(this->darwin_state));
  ach_put(&this->chan_darwin_cmd,        &this->darwin_cmd,        sizeof(this->darwin_cmd));
  ach_put(&this->chan_darwin_cmd_return, &this->darwin_cmd_return, sizeof(this->darwin_cmd_return));
  return;
}

int DarwinAchClient::sleep(double val)
{
  return this->lu->sleep(val);
}

int DarwinAchClient::getState()
{
  size_t fs;
  ach_status_t r = ach_get( &this->chan_darwin_state, &this->darwin_state, sizeof(this->darwin_state), &fs, NULL, ACH_O_LAST );
  return (int)r;
}


int DarwinAchClient::cmd(int cmd)
{
    return this->cmd(cmd,false);
}
int DarwinAchClient::cmd(int cmd, bool block)
{
  size_t fs;
  ach_status_t r = ACH_OK;
  memset(&this->darwin_cmd,   0, sizeof(this->darwin_cmd));
  this->darwin_cmd.cmd = cmd;
  r = ach_put(&this->chan_darwin_cmd, &this->darwin_cmd, sizeof(this->darwin_cmd));

  /* Waits until return of cmd if the a block of "ture" is sent */
  if(block)
  {
    ach_flush(&this->chan_darwin_cmd_return);
    memset(&this->darwin_cmd_return,   0, sizeof(this->darwin_cmd_return));
    ach_status_t r = ach_get( &this->chan_darwin_cmd_return, &this->darwin_cmd_return, sizeof(this->darwin_cmd_return), &fs, NULL, ACH_O_WAIT );
  }
  return (int)r;
}

