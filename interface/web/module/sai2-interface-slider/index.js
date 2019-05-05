(function() {
var template = document.currentScript.ownerDocument.querySelector('#template-slider');

customElements.define('sai2-interface-slider', class extends HTMLElement {
	constructor() {
		super();
		this.template = template;
	}

	connectedCallback() {
		let template_node = this.template.content.cloneNode(true);
		this.key = this.getAttribute('key');
		this.display = this.getAttribute('display');
		this.min = this.getAttribute('min');
		this.max = this.getAttribute('max');
		this.step = this.getAttribute('step');

		// create sliders
		let self = this;
		let container = template_node.querySelector('.container');
		get_redis_val(this.key).done(function(value) {
			// determine iteration bounds: 1 if scalar key, array size if vector
			let parsed_value = JSON.parse(value);
			let len = (Array.isArray(parsed_value)) ? parsed_value.length : 1;

			// save value
			self.value = parsed_value;

			// generate appropriate number of sliders
			for (let i = 0; i < len; i++) {
				/** 
				 * The following js should be equivalent of this:
				 * <div>
				 *   <div class="display_row">
				 * 	   <p class="display"></p>
				 * 	   <p><input type="number" class="number" onkeydown="return false"></p>
				 *   </div>
				 *   <input type="range" class="slider">
				 * <div>
				 */

				let slider_div = document.createElement('div');
				let slider_value_div = document.createElement('div');
				let slider_display = document.createElement('label');
				let slider_value_input = document.createElement('input');
				let slider = document.createElement('input');

				// set up slider name
				slider_display.class = "display";
				if (Array.isArray(parsed_value)){
					slider_display.innerHTML = (self.display || self.key) + "[" + i + "]";
				} else {
					slider_display.innerHTML = self.display || self.key;
				}

				// set up slider value div/container CSS class
				slider_value_div.className = 'display_row';

				// set up manual value input for this slider
				slider_value_input.type = 'number';
				slider_value_input.className = 'number';
				slider_value_input.value = Array.isArray(parsed_value) ? parsed_value[i] : parsed_value;
				slider_value_input.min = (Array.isArray(self.min)) ? self.min[i] : self.min;
				slider_value_input.max = (Array.isArray(self.max)) ? self.max[i] : self.max;
				slider_value_input.step = (Array.isArray(self.step)) ? self.step[i] : self.step;

				// set up typing event
				let typingTimer;
				let sliding_value_input_callback = () => {
					let slider_val = parseFloat(slider_value_input.value);
					if (slider_val < slider_value_input.min)
						slider_val = slider_value_input.min;
					if (slider_val > slider_value_input.max)
						slider_val = slider_value_input.max;

					slider.value = slider_val;

					if (Array.isArray(self.value))
						self.value[i] = slider_val;
					else
						self.value = slider_val;

					post_redis_key_val(self.key, self.value);
				}

				// wait for 250ms for user to stop typing before issuing redis write
				slider_value_input.addEventListener('keyup', () => {
					clearTimeout(typingTimer);
					if (slider_value_input.value)
						typingTimer = setTimeout(sliding_value_input_callback, 250);
				});

				// set up drag slider
				slider.type = 'range';
				slider.className = 'slider';
				slider.value = Array.isArray(parsed_value) ? parsed_value[i] : parsed_value;
				slider.min = (Array.isArray(self.min)) ? self.min[i] : self.min;
				slider.max = (Array.isArray(self.max)) ? self.max[i] : self.max;
				slider.step = (Array.isArray(self.step)) ? self.step[i] : self.step;
				slider.oninput = () => {
					let slider_val = parseFloat(slider.value);
					if (Array.isArray(self.value))
						self.value[i] = slider_val;
					else
						self.value = slider_val;

					slider_value_input.value = slider_val;

					post_redis_key_val(self.key, self.value);
				};

				// append label + manual value input to slider_value_div
				slider_value_div.appendChild(slider_display);
				slider_value_div.appendChild(slider_value_input);
				
				// add them all together
				slider_div.append(slider_value_div);
				slider_div.append(slider);
				container.append(slider_div);
			}
		});

		// append to document
		this.appendChild(template_node);
	}

});
})();