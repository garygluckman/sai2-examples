const template = document.createElement('template');
template.innerHTML = `
  <label class="display"></label>
  <button class="button"></button>
`;


customElements.define('sai2-interface-toggle', class extends HTMLElement {
    constructor() {
      super();
      this.template = template;
      this.current_value = 0;
    }

    connectedCallback() {
      let template_node = this.template.content.cloneNode(true);
      let display_label = template_node.querySelector('.display');
      let button = template_node.querySelector('.button');

      this.key = this.getAttribute('key');
      
      // set display
      display_label.innerHTML = this.getAttribute('display') || this.key;
      get_redis_val(this.key).done((value) => {
          this.current_value = parseInt(value);
          button.innerHTML = this.current_value ? "ON" : "OFF";
          button.className = this.current_value ? "button-enable" : "button-disable";
      });
      
      let self = this;
      // set up listeners
      // register listeners for input textbox
      button.onclick = () => {
          this.current_value = this.current_value ? 0 : 1;
          post_redis_key_val(self.key, this.current_value);
          button.innerHTML = this.current_value ? "ON" : "OFF";
          button.className = this.current_value ? "button-enable" : "button-disable";
      };

      // append to document
      this.appendChild(template_node);
    }
});