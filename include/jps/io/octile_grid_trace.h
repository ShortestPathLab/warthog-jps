#ifndef JPS_IO_OCTILE_GRID_TRACE_H
#define JPS_IO_OCTILE_GRID_TRACE_H

// io/octile_grid_trace.h
//
// Adds support for Octile grid trace, draws successor lines intercardinal then
// cardinal.
//
// @author: Ryan Hechenberger
// @created: 2025-08-07
//

#include <warthog/io/grid_trace.h>

namespace warthog::io
{

/// @brief class that produces a posthoc trace for the gridmap domain, grid
/// must be set.
class octile_grid_trace : public grid_trace
{
public:
	using node = search::search_node;

	using grid_trace::grid_trace;

	void
	print_posthoc_header() override;

protected:
	domain::gridmap* grid_;
};

inline void
octile_grid_trace::print_posthoc_header()
{
	if(*this)
	{
		stream() << R"posthoc(version: 1.4.0
views:
  cell:
    - $: rect
      width: 1
      height: 1
      x: ${{$.x}}
      y: ${{$.y}}
      fill: ${{$.fill}}
      clear: ${{$.clear}}
  succesor:
    - $: cell
      x: ${{$.x}}
      y: ${{$.y}}
      fill: ${{$.fill}}
      clear: ${{$.clear}}
    - $: drawindirect
      $if: ${{ !!parent }}
      x: ${{$.x}}
      y: ${{$.y}}
      dx: ${{$.x-parent.x}}
      dy: ${{$.y-parent.y}}
      fill: ${{$.fill}}
  drawindirect:
    - $: path
      points: [ { x: "${{$.x + 0.5}}", y: "${{$.y  + 0.5}}" },
        { x: "${{ parent.x + 0.5 + ( Math.abs($.dx) < Math.abs($.dy) ? $.dx : Math.sign($.dx) * Math.abs($.dy) ) }}",
          y: "${{ parent.y + 0.5 + ( Math.abs($.dy) < Math.abs($.dx) ? $.dy : Math.sign($.dy) * Math.abs($.dx) ) }}" },
        { x: "${{parent.x + 0.5}}", y: "${{parent.y + 0.5}}" }
        ]
      fill: ${{$.fill}}
      line-width: 0.25
      clear: ${{$.clear}}
  main:
    - $: cell
      $if: ${{ $.type == 'source' }}
      fill: green
      clear: false
    - $: cell
      $if: ${{ $.type == 'destination' }}
      fill: red
      clear: false
    - $: cell
      $if: ${{ $.type == 'expand' }}
      fill: cyan
      clear: false
    - $: cell
      $if: ${{ $.type == 'expand' }}
      fill: blue
      clear: close
    - $: cell
      $if: ${{ $.type == 'generate' }}
      fill: purple
      clear: false
    - $: succesor
      $if: ${{ $.type == 'generate' }}
      fill: orange
      clear: close
pivot:
  x: ${{ $.x + 0.5 }}
  y: ${{ $.y + 0.5 }}
  scale: 1
events:
)posthoc";
	}
}

} // namespace warthog::io

#endif // WARTHOG_IO_GRID_TRACE_H
