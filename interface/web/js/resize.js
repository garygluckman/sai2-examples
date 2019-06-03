
var resize_callbacks = [];
var handle_id = 1;
var init = false;


export function registerWindowResizeCallback(callback) {
  if (!init) {
    window.onresize = () => {
      for (let callback_obj of resize_callbacks) 
        callback_obj.callback();
    };

    init = true;
  }

  resize_callbacks.push({handle: handle_id, callback});
  let ret_handle_id = handle_id;
  handle_id++;
  return ret_handle_id;
}

export function removeWindowResizeCallback(callback_handle) {
  if (!init) 
    return false;

  for (let i = 0; i < resize_callbacks.length; i++) {
    if (resize_callbacks.handle == callback_handle) {
      resize_callbacks.splice(i, 1);
      return true;
    }
  }
  return false;
}
