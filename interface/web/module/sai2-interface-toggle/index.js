// need self executing function, 
// because otherwise template will be declared multiple times
(function() {
    // declare this template outside of constructor
    // because on firefox, document.currentScript does not work 
    // need to query by id, instead of 'template'
    // because on firefox, querying by template results in wrong template
    var template = document.currentScript.ownerDocument.querySelector('#template-toggle-button');
    
    customElements.define('sai2-interface-toggle', class extends HTMLElement {
        constructor() {
            super();
            this.template = template;

            let self = this;
            get_redis_val(this.getAttribute('key')).done(function(value){
                self.current_value = parseInt(value);
            });
        }

        connectedCallback() {
            let template_node = this.template.content.cloneNode(true);
            
            this.key = this.getAttribute('key');
            this.display = this.getAttribute('display');
            
            // set display
            let button = template_node.querySelector('button');
            let displayText = (this.current_value ? 'Disable' : 'Enable' ) + ' ' + this.key;
            button.innerHTML = (this.current_value === null) ? this.key : displayText ;
            
            // set up listeners
            // register listeners for input textbox
            let self = this;
            button.onclick = function(e) {
                post_redis_key_val(self.key, (self.current_value) ? 0 : 1);
                get_redis_val(self.getAttribute('key')).done(function(value){
                    self.current_value = parseInt(value);
                    let displayText = (self.current_value ? 'Disable' : 'Enable' ) + ' ' + self.key;
                    button.innerHTML = (self.current_value === null) ? self.key : displayText;
                });
            };
    
            // append to document
            this.appendChild(template_node);
        }
    });
    })();