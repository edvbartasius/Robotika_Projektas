local machine = require('statemachine')

function onWarn()
    print("Warning!")
end

local fsm = machine.create({
  initial = 'green',
  events = {
    { name = 'warn',  from = 'green',  to = 'yellow' },
    { name = 'panic', from = 'yellow', to = 'red'    },
    { name = 'calm',  from = 'red',    to = 'yellow' },
    { name = 'clear', from = 'yellow', to = 'green'  }
  },
  callbacks = {
    ongreen =  function(self, event, from, to)      print('green light')       end,
    onwarn = onWarn,
    onred =    function(self, event, from, to)      print('red light')         end,
  }
})

fsm:warn()