function debug_printf(level, varargin)
% DEBUG_PRINTF Conditionally print debug statement.
%
%   DEBUG_PRINT(level, message, ...) works just as printf does, except 
%   that debug messages are only printed if 'level' is set to a value 
%   greater than or equal to the value of the global ROVITO_SHOW_DEBUG_LEVEL 
%   variable.  If ROVITO_SHOW_DEBUG_LVL is undefined, no output is printed.
%
%   Inputs:
%   -------
%    level    - Debug message level.
%    message  - Message string, including escape sequences.
%    ...      - Variable-length argument list.

global ROVITO_SHOW_DEBUG_LEVEL;

if level >= ROVITO_SHOW_DEBUG_LEVEL
  disp(sprintf(varargin{:}));
end