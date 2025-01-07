

Button States

- Disabled
  - only while grinding

- Enabled (Default)
  - UI:
    - default state #909090
  - onClick:
    - load associated timer value
    - set ptr to assoc timer value for +/- buttons
    - focus this button

- Focussed
  - UI:
    - highlighted #ffffff
  
  - onClick: (grind current timer value)
    - save state
    - disable all buttons
    - load timer with value
    - attach timer isr to fire every <n> ms
      - decrease timer number
      - decrease arc value
      - allow ui time to update
    - delay 1s or so
    - increment counter for button (i,ii,iii,m)
    - reset arc to 100%
    - reset timer display value to stored value
    - restore state of buttons
    - keep focus on this button

- plus / minus buttons
    - change value 
    - save value for focussed button

- Lock
  - long press for lock / unlock 
  - disable settings button
  - disable +/- buttons

- Settings
  - header: move pages <   [page]   >
  - footer: exit 
  - screens:
    - counters for I, II, III, M
    - brightness
    - colors (white, blue, copper)
    - version
