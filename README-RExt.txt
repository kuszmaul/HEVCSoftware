HEVC_ECF (Extended Chroma Formats)

This model is an extended chroma format version of HM7.0, to include the
chroma formats of 4:2:2, 4:4:4 and 4:0:0. The code has been extensively
changed, and can be summarised as follows:

  1/ TComChromaFormat.h/cpp have been added. These contain small functions
      that control chroma format specific modifications. They are
      generally controlled by macros defined in TypeDef.h.

  2/ TComTU.h/cpp class has been added. This class handles the recursion
      of TUs, which was previously handled in several places in the code.
      
  3/ Debug.h/cpp have been added. These contain some debug functions.
      Please see below for use of environment variables for debugging.
      
  4/ TComRectangle.h has been added.

It is expected that the community will settle on a version of HM that
supports these new formats, and this code is supplied to help reach that
goal. Having different groups working independently to reach that goal may
help produce stable code more quickly, since independent cross-checks
can then be made.
  
For this reason, the code has some configurable aspects, which have been
designed to add flexibility:

  These configurations can be controlled by the ECF macros in
  source/TLibCommon/TypeDef.h, or, if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  is defined as 1, environment variables of the same name can be used to
  save recompiling. Note that debugging functions now also include methods
  to output data from the 'selected-best-decision' in the encoder search.
  
The per-sequence configuration files now indicate their chroma format
(4:2:0 is the default), using the "InputChromaFormat" identifier,
which can be assigned the numerical values of 400, 420, 422 or 444.

The processing format may be different from the InputChromaFormat, with
sub-sampling by dropping samples or sample repeat used to re-sample the
input video. To change the processing format, use -cf XXX 
or --ChromaFormatIDC=XXX on the command line, where XXX is either
400, 420, 422, 444, or 0 (0=use the InputChromaFormat).

The code has been tested against HM7.0 for 4:2:0 and confirmed to match
for the standard test configurations. Some non-standard configurations
(such as changing TU/CU/PU sizes and chroma QP offsets) have also been
recently tested.

Note that there is a macro called ECF__BACKWARDS_COMPATIBILITY_HM7, which
ensures the same behaviour as HM7.0 for some non-standard configurations.
However, it is likely that the intended behaviour is achieved with this
macro set to 0.

